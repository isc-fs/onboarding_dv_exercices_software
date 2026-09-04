"""
IFSSIM Mission Control — FastAPI Backend

Unified API for simulator management, event control, scoring, and telemetry.
Run: uvicorn main:app --host 0.0.0.0 --port 8000
"""

import os
import re
import sys
import glob
import json
import io
import base64
import asyncio
import shutil
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query, Header, HTTPException, Depends, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel

from sim_client import SimConnection
from scoring import compute_scoring

# #465 — bag-record helpers. Pure-Python, no FastAPI coupling, so the
# test suite can drive subprocess calls with mocks. See bag_recorder.py
# for the orchestration contract this module relies on.
import bag_recorder as _bag
from ros_bridge import RosBridge, StartMissionOutcome


# Map the backend's event_type strings (sourced from BUILTIN_TRACKS
# and the EventSetup model — "acceleration"/"skidpad"/"autocross"/
# "trackdrive") to the action-spec mission names accepted by
# mode_manager (VALID_MISSIONS = {trackdrive, autocross, accel,
# skidpad}). The only translation is "acceleration" → "accel"; the
# others pass through. Keeping this mapping at the backend boundary
# rather than aligning the action spec means the user-facing event
# type stays FS-Rules-canonical ("Acceleration") while the wire
# protocol stays compact.

# Wall-clock wait between RuntimeControl (autonomy activate) and EBS
# release. odometry_filter's on_activate runs a stationary bias-
# calibration window (default 3.0 s, see EkfParams::calibration_seconds);
# EBS must stay engaged for the full window or bias absorbs early
# motion and yaw drifts 20–30° in the first few seconds. Tracks the
# EKF default with a small margin for DDS / lifecycle propagation.
_EKF_CAL_WAIT_S = 3.5

# Polling interval during the cal wait. Each sample reads the sim's
# vehicle state so we can prove (in the session log) whether EBS is
# physically pinning the car. If vx stays at zero through the wait,
# the calibration window saw stationary IMU. If vx > 0 mid-wait, EBS
# is letting the chassis roll despite `activateEbs`, and the bring-up
# reorder alone won't fix yaw drift.
_EKF_CAL_POLL_S = 0.5


_EVENT_TO_MISSION = {
    "acceleration": "accel",
    "skidpad":      "skidpad",
    "autocross":    "autocross",
    "trackdrive":   "trackdrive",
}

# Built-in tracks that ship with the simulator — not deletable, auto-configure event type
BUILTIN_TRACKS = {
    "acceleration.csv": "acceleration",
    "skidpad.csv": "skidpad",
}

# Track generator path
TRACK_GEN_PATH = os.path.abspath(os.environ.get("TRACK_GEN_PATH",
    os.path.join(os.path.dirname(__file__), "..", "..", "random-track-generator")))
TRACKS_DIR = os.path.abspath(os.environ.get("TRACKS_DIR",
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "Content", "tracks")))
PIPELINE_CTL_FILE = "/pipeline_ctrl/enable"
# Path UE5 uses to load the file — must be the host-side absolute path (UE5 runs on host, not in Docker)
UE5_TRACKS_DIR = os.environ.get("UE5_TRACKS_DIR", TRACKS_DIR)

# Insert TRACK_GEN_PATH into sys.path ONCE at module load (#327 B9).
# Pre-#327 `track_generate` did `sys.path.insert(0, TRACK_GEN_PATH)`
# every call without ever removing it — N-th call left N copies on
# the path, and only the first one mattered because Python caches
# imports after the first one. Doing this here means the cost is
# paid once, the imports below resolve cleanly the first time, and
# `track_generate` no longer mutates `sys.path`.
if TRACK_GEN_PATH not in sys.path:
    sys.path.insert(0, TRACK_GEN_PATH)

SIM_HOST = os.environ.get("IFSSIM_HOST", os.environ.get("SIM_HOST", "127.0.0.1"))
SIM_PORT = int(os.environ.get("IFSSIM_PORT", os.environ.get("SIM_PORT", "41451")))

# Optional API key. When set, every mutating endpoint (sim control, event
# control, RES, pipeline, track load/gen/delete) and the telemetry WS
# require a matching key. Read-only status endpoints stay open so a
# monitor dashboard can attach without the key. When unset, the backend
# logs a single warning and runs wide-open — fine for closed-network dev,
# not for track day on untrusted wifi.
MC_API_KEY = os.environ.get("IFSSIM_MC_API_KEY", "").strip()

# CORS origins. Comma-separated list; default restricts to the frontend's
# docker-compose-published address. Use the old wildcard `*` explicitly
# if an external tool needs it — never silently anymore.
_DEFAULT_CORS = "http://localhost:3000,http://127.0.0.1:3000"
MC_CORS_ORIGINS = [o.strip() for o in
    os.environ.get("IFSSIM_MC_CORS_ORIGINS", _DEFAULT_CORS).split(",")
    if o.strip()]

app = FastAPI(title="IFSSIM Mission Control", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=MC_CORS_ORIGINS,
    allow_methods=["*"],
    allow_headers=["*"],
)

if not MC_API_KEY:
    print("[mc] IFSSIM_MC_API_KEY is empty — running unauthenticated. "
          "Set it before exposing the backend beyond localhost.",
          file=sys.stderr, flush=True)


def require_api_key(x_api_key: Optional[str] = Header(default=None)):
    """FastAPI dependency — validates X-API-Key header against MC_API_KEY.
    No-op if the server wasn't configured with a key (dev mode)."""
    if not MC_API_KEY:
        return
    if x_api_key != MC_API_KEY:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="missing or invalid X-API-Key",
        )


def ws_api_key_ok(api_key: Optional[str]) -> bool:
    """WebSocket auth check. Browsers can't attach custom headers to
    a WebSocket handshake, so WS uses a query-param token instead."""
    if not MC_API_KEY:
        return True
    return api_key == MC_API_KEY

sim = SimConnection(SIM_HOST, SIM_PORT)

# State
# `session_log` was an unbounded list pre-#327 (B3). Multi-hour
# track-day sessions with frequent RES toggles, resets, pipeline
# starts/stops, track loads, and event_starts accumulated thousands
# of entries — all returned in full on every `/api/session/log`
# poll (3 s tick from the frontend, see #326 F5 fix) and serialised
# on every export. 5000 entries is generous: at one event per
# second of active operation that's ~80 minutes of dense activity,
# beyond which the oldest entries fall off automatically.
_SESSION_LOG_MAX = 5000
session_log: deque = deque(maxlen=_SESSION_LOG_MAX)
res_active = False
home_pose = {"x": 0.0, "y": 0.0, "z": 0.3, "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0}  # ENU spawn pose
_home_pose_captured = False  # set once we snapshot the pawn's map placement (or user pins a home)
_sim_was_connected = False   # tracks connect/disconnect transitions so we re-capture after a UE5 restart
# Idempotency guard for `event_start`. Pre-#327 a double-click on
# Start Session would block 4.5 s on `_state_lock`, then immediately
# re-run the entire boot sequence — duplicate RES toggles, duplicate
# pipeline restarts, duplicate `set_event` calls (#327 B7). Frontend
# now disables the Start button while in flight (#326 F7) but the
# server-side guard is the load-bearing one. Protected by
# `_state_lock`.
_session_starting = False

# Compound-operation lock. FastAPI runs sync `def` handlers in a worker
# thread pool, so two clients can land in `event_start`, `sim_reset` or
# the RES endpoints simultaneously. Each of those does N sequential
# RPCs *plus* mutates `res_active` / `current_event` / `PIPELINE_CTL_FILE`,
# and we don't want one handler to interleave with another mid-update
# (visible as RES toggle flicker, ghost pipeline restarts, and a
# split-brain `res_active` flag). All wire-level RPC calls go through
# `SimConnection._lock` already, so this lock is purely for the
# *compound* state.
_state_lock = threading.Lock()

# `track_generate` calls `os.chdir` (the third-party generator uses
# `__file__`-relative paths internally). chdir is process-global, so a
# concurrent track-generate would clobber the other's CWD, and any
# unrelated handler that runs in between would see the wrong CWD. The
# generator is also cpu-heavy, so we don't want to share `_state_lock`
# with the fast RES endpoints. Dedicated lock.
_gen_lock = threading.Lock()

# Home-pose / sim-connect-tracking globals (#327 B5). Pre-#327 these
# were mutated and read from FastAPI's threadpool with no
# synchronization. CPython's GIL prevents single-attribute tearing,
# but the multi-field `home_pose = {...}` assignment was observable
# mid-state by a reader from `sim_status` / `vehicle_state`, and the
# `_home_pose_captured` flag could desync from the dict. Dedicated
# lock so we don't contend with `_state_lock` (RES toggles fire
# every frame on a Stop-Session click). Brief enough that a
# read-then-act pattern under the lock is fine.
_home_lock = threading.Lock()


def _ensure_home_pose_captured():
    """Snapshot the pawn's map-placement pose as home.

    Runs from the polled status/state endpoints so the capture happens within the
    first poll after UE5 connects — before the pawn has had time to drive away
    from its spawn. On UE5 disconnect (Stop/Play cycle), the captured flag is
    reset so the next connect re-captures fresh.

    All reads/writes to `home_pose`, `_home_pose_captured`, and
    `_sim_was_connected` are now under `_home_lock` (#327 B5).
    The actual RPC (`sim.get_vehicle_pose`) is called WITHOUT the
    lock — it's the only slow part of this function, and the inner
    `SimConnection._lock` already protects the wire.
    """
    global home_pose, _home_pose_captured, _sim_was_connected
    is_conn = sim.is_connected()

    # Phase 1 (under `_home_lock`): update connection-tracking state
    # and decide whether we need to capture this tick.
    with _home_lock:
        if _sim_was_connected and not is_conn:
            _home_pose_captured = False
        _sim_was_connected = is_conn
        need_capture = is_conn and not _home_pose_captured
    if not need_capture:
        return

    # Phase 2 (no lock): RPC for the pose.
    try:
        pose = sim.get_vehicle_pose()
    except Exception:
        return
    if not pose:
        return
    new_home = {
        "x": pose.get("x", 0.0),
        "y": pose.get("y", 0.0),
        "z": 0.3,  # fixed lift so reset never spawns at ground level
        "qw": pose.get("qw", 1.0),
        "qx": pose.get("qx", 0.0),
        "qy": pose.get("qy", 0.0),
        "qz": pose.get("qz", 0.0),
    }

    # Phase 3 (under `_home_lock`): atomic publish of dict + flag.
    # Re-check `_home_pose_captured` inside the lock — a concurrent
    # caller may have captured between Phase 1 and here. Last writer
    # wins (the values are equivalent within a few ms of each other,
    # so there's no semantic conflict).
    with _home_lock:
        home_pose = new_home
        _home_pose_captured = True
_STATE_FILE = os.path.join(TRACKS_DIR, ".ifssim_state.json")

import logging as _logging  # noqa: E402 — co-located with the helpers that use it

_state_logger = _logging.getLogger("mission_control.state")


def _load_state():
    try:
        with open(_STATE_FILE) as f:
            return json.load(f)
    except FileNotFoundError:
        # First run, or state file deleted on purpose. Not an error.
        return {}
    except Exception as e:
        # Any other failure — corrupt JSON, permission denied, full
        # disk on a read of the atime — was silently swallowed
        # pre-#327 (B10). Now logged so a wedged state file doesn't
        # disappear into the void.
        _state_logger.warning("state file load failed: %s", e)
        return {}


def _save_state(data: dict):
    """Write `data` (merged with existing state) atomically.

    Pre-#327 (B10) this opened the file for write directly and on a
    crash between truncate and `json.dump` left a zero-byte file;
    next startup `_load_state` swallowed the resulting JSONDecodeError
    and the user silently lost their last selected event. Now writes
    to `<state>.tmp` first, then `os.replace`s into place — the
    rename is atomic on POSIX and on NTFS, so any read either sees
    the old file or the fully-written new file, never partial.
    """
    try:
        s = _load_state()
        s.update(data)
        tmp_path = _STATE_FILE + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(s, f)
            # Force the OS to flush the user buffer. Without this,
            # a crash between `close` and `replace` could land us in
            # the same state we were trying to avoid — except the
            # `.tmp` file is the corrupt one and the original is
            # still sound, so cost-of-fix-on-crash is at worst the
            # next save that overwrites the bad tmp.
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, _STATE_FILE)
    except Exception as e:
        _state_logger.warning("state file save failed: %s", e)

current_event = _load_state().get("event", "unknown")

# #465 — last-loaded track name (basename), updated by track_load.
# The bag-record name builder consumes it; "no-track" is the safe
# fallback (which also gates the existing "refuse start with no
# track" check in event_start, so we never compose a name from
# stale data after a fresh sim boot).
current_track: Optional[str] = None

# #465 — live bag-record state. None when no recording is active;
# otherwise a dict with keys `name`, `state`, `path`, `error`. Mutated
# only under _state_lock so concurrent endpoints (event_start from
# one HTTP worker + pipeline_stop from another) can't race the
# lifecycle transitions.
#
# Note: the actual `ros2 bag record` subprocess lives in
# dv_pipeline_stack now (#465 v2). The backend interacts with it
# only through the /bag_recorder/{start,stop} services; this dict
# holds the latest service-response snapshot, NOT process state.
_active_recording: Optional[dict] = None


# === Pydantic Models ===

class EventSetup(BaseModel):
    event_type: str
    num_laps: int = 10
    # #465 — optional bag-record toggle. Default False so a missing or
    # legacy client (UI without the checkbox, or a script call) never
    # silently fills disk. When True, event_start kicks off
    # `ros2 bag record -s mcap -a -o /bags/<name>` inside the dv
    # pipeline container after autonomy is up, and pipeline_stop /
    # any equivalent session-stop tears it down + copies the bag out.
    record_bag: bool = False

class TrackGenerate(BaseModel):
    n_points: int = 50
    n_regions: int = 30
    max_bound: int = 150
    name: str = ""

class TeleportRequest(BaseModel):
    x: float
    y: float
    z: float


# === Sim Status ===

@app.get("/api/sim/status")
def sim_status():
    connected = sim.is_connected()
    status = sim.get_status() if connected else {}
    _ensure_home_pose_captured()
    return {
        "connected": connected,
        "map": status.get("map", "unknown"),
        "fps": status.get("fps", 0),
        "paused": status.get("paused", False),
        "api_control": status.get("api_control", False),
    }

@app.post("/api/sim/pause", dependencies=[Depends(require_api_key)])
def sim_pause():
    try:
        sim.pause()
        return {"ok": True, "paused": True}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)

@app.post("/api/sim/resume", dependencies=[Depends(require_api_key)])
def sim_resume():
    try:
        sim.resume()
        return {"ok": True, "paused": False}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)

@app.post("/api/sim/reset", dependencies=[Depends(require_api_key)])
def sim_reset():
    """Soft reset: teleport car back to start position without crashing UE5.
    The destructive RPC 'reset' triggers a full level reload which crashes the
    UE5 editor. Use teleport + disable API control instead."""
    global res_active
    if not sim.is_connected():
        # Pre-#327 this returned 200 OK with `{"ok": false}`, so any
        # frontend or middleware switching on HTTP status thought the
        # reset had succeeded (B6). Now correctly 503 Service
        # Unavailable; body shape preserved so existing frontend
        # `r.ok ? success : error` paths still work.
        return JSONResponse({"ok": False, "error": "sim not connected"}, status_code=503)
    with _state_lock:
        _ensure_home_pose_captured()
    # Stop the autonomy outside the state lock — the action call
    # blocks for tens of ms while mode_manager fans out
    # change_state(DEACTIVATE+CLEANUP), and we don't want to hold
    # the state lock during DDS round-trips. Idempotent post-#379;
    # a no-op if autonomy isn't active. Skipped entirely when
    # ros_bridge isn't available (legacy flag-file path retired
    # in #381).
    if _ros_bridge_available:
        try:
            RosBridge.get().stop_mission()
        except Exception as ex:
            log_event("sim_reset",
                      f"autonomy stop failed (continuing reset anyway): {ex}")
    # Resolve the target pose to teleport to. Priority order:
    #
    #   1. The plugin's stored start-gate pose (LastStartGateLoc_UE /
    #      LastStartGateRot_UE), exposed via the `getStartGatePose`
    #      RPC. This is the SAME pose `loadTrack` aligns the car to,
    #      so "/api/sim/reset" becomes semantically "redo the car-
    #      alignment step of the last loadTrack" — no cone respawn,
    #      no scene rebuild, just the car. This is the authoritative
    #      target whenever a track has been loaded.
    #
    #   2. Captured `home_pose` as fallback. Used only when no track
    #      has been loaded yet (e.g. first sim connect on customMap
    #      with no track) — in that regime the only "home" we know
    #      is the PlayerStart pose snapshotted at first connect.
    #
    # Previous behaviour: read x/y/z from home_pose, DROP the captured
    # quaternion, call teleport_pos (position-only). Two compounding
    # bugs:
    #   a) `home_pose` was captured at sim-connect, before the user
    #      loaded a track. So its quaternion was PlayerStart's
    #      (typically yaw=+90°), never the track's start gate.
    #   b) Even if (a) were right, dropping the quaternion meant the
    #      plugin's position-only handler restored the pawn's CURRENT
    #      orientation — fine before a drive, wrong after one.
    # Net result: post-drive resets returned the car to spawn XY but
    # with whatever yaw it had when reset was pressed. Visible as the
    # autonomy launching at the wrong heading on the first tick.
    start_gate = None
    try:
        start_gate = sim.get_start_gate_pose()
    except Exception as ex:
        log_event("sim_reset",
                  f"get_start_gate_pose failed (falling back to home_pose): {ex}")

    if start_gate is not None:
        tx = start_gate["x"]
        ty = start_gate["y"]
        tz = start_gate["z"]
        qw = start_gate["qw"]
        qx = start_gate["qx"]
        qy = start_gate["qy"]
        qz = start_gate["qz"]
        target_source = "start_gate"
    else:
        with _state_lock:
            # Snapshot the home pose under `_home_lock` (#327 B5) so
            # the seven coordinates we pass to teleport are consistent
            # with each other — a concurrent capture could otherwise
            # update fields one at a time and we'd teleport to a mid-
            # update pose.
            with _home_lock:
                tx = home_pose["x"]
                ty = home_pose["y"]
                tz = home_pose["z"]
                qw = home_pose.get("qw", 1.0)
                qx = home_pose.get("qx", 0.0)
                qy = home_pose.get("qy", 0.0)
                qz = home_pose.get("qz", 0.0)
        target_source = "home_pose"

    with _state_lock:
        try:
            sim.res_activate()
            # Full pose teleport (xyz + quaternion). The plugin's
            # simSetVehiclePose handler does the same teleport-with-
            # velocity-reset sequence loadTrack uses for the car —
            # SetActorLocationAndRotation, body-transform snap,
            # zero linear + angular velocities, ResetVehicleState —
            # so once we hand it the right pose, post-reset state
            # matches post-loadTrack state for the car.
            sim.teleport(tx, ty, tz, qw, qx, qy, qz)
            res_active = False
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
        log_event(
            "reset",
            f"Soft reset: pipeline stopped, car teleported to {target_source} "
            f"({tx:.2f}, {ty:.2f}, {tz:.2f})")
    return {"ok": True, "target": target_source}


# === Event Control ===

@app.get("/api/event/state")
def event_state():
    try:
        state = sim.get_referee_state() or {}
    except Exception:
        state = {}
    # #465 — surface live bag-record state alongside the referee
    # state. Both serialise cheap fields only — no subprocess on the
    # hot path; the recorder's PID and bag path are populated exactly
    # once (at start / stop) by the lifecycle handlers.
    with _state_lock:
        rec = _active_recording
        if rec:
            state["bag_name"] = rec.get("name")
            state["bag_state"] = rec.get("state")
            if rec.get("path"):
                state["bag_path"] = rec["path"]
            if rec.get("error"):
                state["bag_error"] = rec["error"]
        else:
            state["bag_state"] = "none"
    return state

@app.post("/api/event/set", dependencies=[Depends(require_api_key)])
def event_set(setup: EventSetup):
    global current_event
    try:
        result = sim.set_event(setup.event_type, setup.num_laps)
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
    current_event = setup.event_type
    _save_state({"event": current_event})
    log_event("event_set", f"{setup.event_type} ({setup.num_laps} laps)")
    return result

@app.post("/api/event/start", dependencies=[Depends(require_api_key)])
def event_start(setup: EventSetup):
    """Boot the autonomy pipeline (SetMission then RuntimeControl).

    Phases:
      1. Pre-flight (under `_state_lock`): connectivity check, cone
         preflight, RES-activate to park the car, set event type,
         resume sim.
      2. Prepare (NO LOCK): `RosBridge.get().set_mission(mission)`.
         Supervisor → mission_control → mode_manager CONFIGURE only
         (Numba JIT, SLAM setup, etc.) while EBS holds the car still.
      3. Run (NO LOCK): `RosBridge.get().start_runtime()` opens
         RuntimeControl on mission_control (ACTIVATE). EBS is STILL
         ENGAGED — odometry_filter's on_activate kicks its 3 s
         stationary bias-calibration window with the car physically
         pinned, which is what makes the bias estimate valid.
      3.5. Sleep `_EKF_CAL_WAIT_S` for the EKF calibration window to
         finish, with EBS still holding the car still. Polls the sim
         every `_EKF_CAL_POLL_S` and logs vx so we can prove (in the
         session log) whether EBS actually pins the chassis. Without
         this wait, Phase 4 races the calibration accumulator and
         the EKF absorbs early motion into bias (see #518 post-mortem
         on bag autocross_no-track_20260517_202133 — 22–32° yaw drift).
      4. Release EBS + enable API control (under `_state_lock`).
         Control commands now flow via action feedback; sim_supervisor
         relays to /fsds/control_command for the UE5 bridge.

    Pre-#518 the action chain bundled CONFIGURE + ACTIVATE in a single
    goal, so the original EBS-then-activate ordering was fine — there
    was no separate "run" step. #518 split prepare from run but left
    EBS release in slot 3, which silently put activate AFTER EBS
    release. This restores the correct ordering. When ros_bridge isn't
    available — unit-test envs without rclpy — the legacy flag-file
    fallback is retained so existing tests don't have to spin up DDS.

    Idempotency: `_session_starting` flag prevents double-click
    re-firing the boot sequence (B7). Returns 409 if a start is
    already in progress.
    """
    global current_event, res_active, _session_starting
    if not sim.is_connected():
        return JSONResponse({"ok": False, "error": "Simulator not connected"}, status_code=503)
    # Refuse to start a session if no track is loaded into UE5. Without
    # cones the autonomy stack starts from a blank world: SLAM sees no
    # landmarks, the planner gets no /Conos, the controller publishes
    # zero, and the car creeps forward (via residual EMRAX idle torque
    # or just gravity) under the impression "everything is fine". The
    # user's expectation when track-load silently fails is that nothing
    # happens, not that the car drives.
    try:
        ref = sim.get_referee_state()
        cones = ref.get("cones", 0) if isinstance(ref, dict) else 0
    except Exception:
        cones = 0
    if cones <= 0:
        return JSONResponse(
            {"ok": False, "error": "No track loaded — load a track before starting a session"},
            status_code=400,
        )

    # B7 — idempotency. Claim the start slot under the lock; if
    # another call is already in flight, refuse with 409 Conflict.
    with _state_lock:
        if _session_starting:
            return JSONResponse(
                {"ok": False, "error": "Another session start is already in progress"},
                status_code=409,
            )
        _session_starting = True

    try:
        # === Phase 1: pre-flight sim-side setup (under _state_lock) ===
        with _state_lock:
            try:
                # Park the car under EBS while the autonomy boots.
                # Mode_manager's CONFIGURE step on cone_detection_node
                # runs the Numba JIT compile (10–20 s); odometry_filter's
                # on_activate (Phase 3) then runs a ~3 s stationary
                # bias-calibration window. EBS stays engaged through
                # both AND through the Phase 3.5 wait, so the EKF
                # always calibrates on truly motionless IMU.
                sim.res_activate()
                res_active = True
                sim.set_event(setup.event_type, setup.num_laps)
                sim.resume()
            except Exception as e:
                return JSONResponse({"ok": False, "error": str(e)}, status_code=500)

        # === Phase 2: prepare autonomy via SetMission (NO LOCK) ===
        if _ros_bridge_available:
            mission = _EVENT_TO_MISSION.get(setup.event_type, "trackdrive")
            if setup.event_type not in _EVENT_TO_MISSION:
                log_event(
                    "event_start",
                    f"unknown event {setup.event_type!r}; defaulting mission "
                    f"to 'trackdrive'",
                )
            log_event(
                "event_start",
                f"SetMission(mission={mission!r}) — configure only, "
                f"can take 10–20 s",
            )
            prep = RosBridge.get().set_mission(mission)
            if not prep.success:
                with _state_lock:
                    try:
                        sim.res_activate()
                        res_active = True
                    except Exception:
                        pass
                log_event(
                    "event_start",
                    f"SetMission failed: {prep.message}",
                )
                return JSONResponse(
                    {
                        "ok": False,
                        "error": f"autonomy prepare failed: {prep.message}",
                    },
                    status_code=502,
                )
        else:
            # ros_bridge unavailable — only happens when the backend
            # is running outside the mission_control_backend container
            # (e.g. local dev with no rclpy installed). The action
            # chain is the canonical path post-#381; refuse with 503
            # rather than fall back to the retired flag-file mechanism.
            log_event(
                "event_start",
                "ros_bridge unavailable; autonomy bring-up requires "
                "the action chain (the flag-file fallback was retired in #381)",
            )
            return JSONResponse(
                {
                    "ok": False,
                    "error": "ros_bridge not initialised — autonomy bring-up unavailable",
                },
                status_code=503,
            )

        # === Phase 3: activate + run via RuntimeControl (NO LOCK) ===
        #
        # ORDER MATTERS. Activate runs BEFORE EBS release so
        # odometry_filter's on_activate kicks its 3 s stationary
        # bias-calibration window with the car physically pinned by
        # EBS. Pre-#518 the action chain bundled CONFIGURE + ACTIVATE
        # in Phase 2 (single goal), so the original EBS-then-run
        # ordering was fine — there was no separate "run" step. #518
        # split prepare from run but left EBS release in slot 3,
        # which silently put activate AFTER EBS release. That broke
        # the calibration window's stationarity precondition and
        # introduced the 22-32° yaw drift seen in all post-#518 bags.
        if _ros_bridge_available:
            log_event(
                "event_start",
                "RuntimeControl — activating autonomy stack (EBS still engaged)",
            )
            runtime = RosBridge.get().start_runtime()
            if not runtime.success:
                # EBS is still active here, so no rollback needed —
                # just leave the latch and surface the failure.
                log_event(
                    "event_start",
                    f"RuntimeControl failed: {runtime.message}",
                )
                return JSONResponse(
                    {
                        "ok": False,
                        "error": f"autonomy start failed: {runtime.message}",
                    },
                    status_code=502,
                )

            # === Phase 3.5: wait for EKF calibration, log vx ===
            #
            # We poll the sim's vehicle state every _EKF_CAL_POLL_S so
            # the session log records whether EBS is actually pinning
            # the chassis during the calibration window. If vx stays
            # at 0, the EKF calibrated on stationary IMU — Track 1 of
            # the post-#518 fix is working. If vx rises mid-wait, the
            # `activateEbs` UE5 plugin call isn't physically stopping
            # the car (it may just be locking control inputs) and the
            # fix needs to happen one layer down in the plugin.
            log_event(
                "event_start",
                f"waiting {_EKF_CAL_WAIT_S:.1f}s for EKF stationary "
                f"bias-calibration window (EBS still engaged)",
            )
            _speed_samples: list[float] = []
            _wait_start = time.monotonic()
            while (time.monotonic() - _wait_start) < _EKF_CAL_WAIT_S:
                try:
                    state = sim.get_vehicle_state()
                    # `speed` is |velocity| from getCarState — frame
                    # agnostic, what we actually care about.
                    _speed_samples.append(float(state.get("speed", 0.0)))
                except Exception:
                    # Don't fail the boot on a transient sim RPC blip;
                    # just skip this sample.
                    _speed_samples.append(float("nan"))
                time.sleep(_EKF_CAL_POLL_S)
            _max_speed = max((v for v in _speed_samples if v == v), default=0.0)
            _ebs_held = _max_speed < 0.05  # 5 cm/s threshold for "still"
            log_event(
                "event_start",
                f"EKF cal wait done | max speed during wait = "
                f"{_max_speed:.3f} m/s | EBS_held={_ebs_held} | "
                f"samples={[round(v, 3) for v in _speed_samples]}",
            )

        # === Phase 4: release EBS (under lock) ===
        with _state_lock:
            try:
                sim.res_release()
                res_active = False
                try:
                    sim._cmd("enableApiControl 1")
                except Exception:
                    pass
            except Exception as e:
                return JSONResponse({"ok": False, "error": str(e)}, status_code=500)

        with _state_lock:
            current_event = setup.event_type
            _save_state({"event": current_event})
            # The plugin's SetEventType clamps lap counts per event
            # type (Acceleration/Autocross → 1, Skidpad → 4,
            # Trackdrive → requested). Echo the referee's actual
            # required_laps so the UI reflects what the sim will
            # enforce, not what we asked for.
            try:
                ref = sim.get_referee_state()
                actual_laps = int(ref.get("required_laps", setup.num_laps))
            except Exception:
                actual_laps = setup.num_laps
            log_event("event_start", f"{setup.event_type} started ({actual_laps} laps)")

            # #465 — optional bag-record. Strictly post-autonomy-up so
            # the recording window matches the lap, not the
            # autonomy-warmup phase. The actual `ros2 bag record`
            # process lives inside dv_pipeline_stack now (#465 v2);
            # we just call /bag_recorder/start over DDS. _state_lock
            # serialises against pipeline_stop's matching teardown.
            global _active_recording
            bag_info = None
            if setup.record_bag:
                if _active_recording and _active_recording.get("state") == "recording":
                    log_event(
                        "record_bag",
                        f"prior recording {_active_recording.get('name')} "
                        "still active — closing before new start",
                    )
                    try:
                        _bag.request_stop(RosBridge.get())
                    except Exception as ex:
                        log_event("record_bag", f"prior-stop failed: {ex}")
                    _active_recording = None

                bag_name = _bag.compose_bag_name(setup.event_type, current_track)
                start_resp = _bag.request_start(RosBridge.get(), bag_name)
                if start_resp.get("ok"):
                    _active_recording = {
                        "name": bag_name,
                        "state": start_resp.get("state", "recording"),
                        "path": start_resp.get("path", ""),
                    }
                    bag_info = {
                        "name": bag_name,
                        "state": _active_recording["state"],
                    }
                    log_event(
                        "record_bag",
                        f"started {bag_name} → {start_resp.get('path')}",
                    )
                else:
                    # Service-call failed or recorder refused (disk full,
                    # spawn error, …). Do NOT fail the session —
                    # autonomy is already running and the operator can
                    # decide whether to retry.
                    _active_recording = None
                    bag_info = {
                        "error": start_resp.get("error", "start failed"),
                        "state": "failed",
                    }
                    log_event(
                        "record_bag",
                        f"start failed: {start_resp.get('error')}",
                    )

        response = {
            "ok": True,
            "event": setup.event_type,
            "laps": actual_laps,
            "autonomy_via": "ros_action" if _ros_bridge_available else "flag_file_legacy",
        }
        if bag_info is not None:
            response["bag"] = bag_info
        return response
    finally:
        # Always clear the idempotency flag — success, exception,
        # interrupt-abort, or 5xx error from the inner try. Without
        # this a transient failure would wedge `_session_starting=True`
        # forever and every subsequent Start would 409.
        with _state_lock:
            _session_starting = False


# === RES (Remote Emergency Stop) ===

@app.post("/api/res/activate", dependencies=[Depends(require_api_key)])
def res_activate_endpoint():
    global res_active
    with _state_lock:
        # RES = engage emergency brake. Pipeline (SLAM, planner, controller)
        # KEEPS RUNNING — they still see incoming sensor data, the planner
        # keeps producing paths, the control node keeps publishing
        # /control_command. The bridge drops /control_command silently
        # while ebs_triggered_ is set (ifssim_ros_wrapper.cpp:1023), so no
        # actuator output reaches UE5 until the user releases RES.
        #
        # Deleting the pipeline flag here was a regression: it caused the
        # autonomy nodes to be killed by the entrypoint loop, requiring a
        # full SLAM re-init (and another 4.5 s IMU calibration) on
        # release. Toggling RES mid-session is now reversible — release
        # the brake and the autonomy resumes from where it was.
        try:
            sim.res_activate()
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
        res_active = True
        log_event("res", "EMERGENCY STOP activated")
    return {"ok": True, "res": "activated", "res_active": True}

@app.post("/api/res/release", dependencies=[Depends(require_api_key)])
def res_release_endpoint():
    global res_active
    with _state_lock:
        # Re-check under the lock — without it, an `event_start` running
        # concurrently could clear the flag between the check and the
        # release, and we'd issue a redundant `releaseEbs` to the sim.
        if not res_active:
            return JSONResponse(
                {"ok": False, "error": "RES is not active"},
                status_code=400,
            )
        try:
            sim.res_release()
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
        res_active = False
        log_event("res", "RES released")
    return {"ok": True, "res": "released", "res_active": False}

@app.get("/api/res/status")
def res_status():
    # `res_active` is a single bool, so the read is atomic under the
    # GIL — but during a RES toggle the writer is mid-RPC and the
    # value's eventual state may differ from what the reader sees.
    # Holding `_state_lock` for the read serialises us behind the
    # in-flight toggle, so the response reflects the post-toggle
    # state instead of a pre-toggle snapshot (#327 B5).
    with _state_lock:
        return {"res_active": res_active}


# === Pipeline ===

@app.post("/api/pipeline/start", dependencies=[Depends(require_api_key)])
def pipeline_start():
    """Bring the autonomy pipeline to `active` via StartMission.

    Calls sim_supervisor_node.StartMission with the current event's
    mission. The supervisor relays to mission_control_node, which
    calls mode_manager.activate_mode, which fans out change_state
    across every autonomy LifecycleNode. Blocks until the supervisor
    reports ready/failed (typically 10–25 s, dominated by Numba JIT
    in cone_detection_node's on_configure).

    Mode_manager's _drive_transition is idempotent post-#379 — calling
    /api/pipeline/start when autonomy is already active is a no-op
    that returns success immediately.

    Mission selection: derived from `current_event` via
    _EVENT_TO_MISSION. Defaults to "trackdrive" when the current
    event is unknown — matches the most common dev scenario.

    Pre-#381 a flag-file fallback existed for envs without rclpy;
    that path was retired alongside entrypoint.sh's polling loop.
    """
    if not _ros_bridge_available:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=(
                "ros_bridge not initialised — autonomy bring-up requires "
                "the StartMission action chain (the flag-file path was "
                "retired in #381)"
            ),
        )

    with _state_lock:
        event_snap = current_event
    mission = _EVENT_TO_MISSION.get(event_snap, "trackdrive")
    if event_snap not in _EVENT_TO_MISSION:
        log_event(
            "pipeline",
            f"unknown event {event_snap!r}; defaulting mission "
            f"to 'trackdrive'",
        )

    log_event(
        "pipeline",
        f"SetMission(mission={mission!r}) — prepare only (10–25 s)",
    )
    outcome = RosBridge.get().set_mission(mission)

    if not outcome.success:
        log_event("pipeline", f"SetMission failed: {outcome.message}")
        raise HTTPException(
            status_code=status.HTTP_502_BAD_GATEWAY,
            detail=outcome.message,
        )

    log_event("pipeline", f"SetMission ready (mission={mission!r})")
    return {
        "ok": True,
        "pipeline": "started",
        "mission": mission,
        "via": "ros_action",
        "message": outcome.message,
    }

@app.post("/api/pipeline/stop", dependencies=[Depends(require_api_key)])
def pipeline_stop():
    """Tear down: cancel RuntimeControl, then SetMission(mission_id=0)."""
    if not _ros_bridge_available:
        # No flag-file fallback post-#381; nothing to stop if rclpy
        # isn't available (the autonomy can only run via the action
        # chain which requires ros_bridge).
        log_event("pipeline", "Pipeline stop: ros_bridge unavailable; no-op")
        return {"ok": True, "pipeline": "stopped", "via": "no_op"}

    outcome = RosBridge.get().stop_mission()
    if not outcome.success:
        log_event("pipeline", f"Pipeline stop failed: {outcome.message}")
        raise HTTPException(
            status_code=status.HTTP_502_BAD_GATEWAY,
            detail=outcome.message,
        )

    # #465 — tear down any in-flight recording before reporting back.
    # Done AFTER the autonomy stop so any final messages flushed by
    # cone_detection/slam/control during their on_deactivate land in
    # the bag rather than getting cut off mid-tick. Stop is best-
    # effort — pipeline stop's success is not contingent on it.
    global _active_recording
    bag_info = None
    with _state_lock:
        if _active_recording and _active_recording.get("state") in (
            "recording", "starting",
        ):
            name = _active_recording.get("name")
            stop_resp = _bag.request_stop(RosBridge.get())
            _active_recording = {
                "name": name,
                "state": stop_resp.get("state", "failed"),
                "path": stop_resp.get("path", _active_recording.get("path", "")),
            }
            if stop_resp.get("error"):
                _active_recording["error"] = stop_resp["error"]
            bag_info = {
                "name": name,
                "state": _active_recording["state"],
                "path": _active_recording.get("path", ""),
            }
            if _active_recording.get("error"):
                bag_info["error"] = _active_recording["error"]
                log_event(
                    "record_bag", f"stop failed for {name}: {bag_info['error']}",
                )
            else:
                log_event(
                    "record_bag",
                    f"stopped {name} → {bag_info['path']}",
                )
                # #498 — auto-pull the finalised bag onto the host
                # filesystem (and clean from the docker volume).
                # Best-effort: a pull failure does NOT fail the
                # pipeline-stop; the bag is still safe in the
                # volume and the user can recover via
                # `tools/pull-bag.sh`.
                if _bag.is_auto_pull_enabled() and stop_resp.get("ok"):
                    pull = _bag.auto_pull_and_clean(name)
                    if pull.get("ok"):
                        bag_info["host_path"] = pull["host_path"]
                        _active_recording["host_path"] = pull["host_path"]
                        if pull.get("error"):
                            # Cleanup partial-failure path: bag is on host,
                            # but the volume-side rm failed. Surface as a
                            # warning, don't taint the bag_info["state"].
                            log_event(
                                "record_bag",
                                f"auto-pulled {name} → {pull['host_path']} "
                                f"(warning: {pull['error']})",
                            )
                        else:
                            log_event(
                                "record_bag",
                                f"auto-pulled {name} → {pull['host_path']}",
                            )
                    else:
                        # Pull failed: bag stays in volume. Log it so
                        # the user knows to retrieve manually.
                        bag_info["pull_error"] = pull.get(
                            "error", "unknown auto-pull failure",
                        )
                        log_event(
                            "record_bag",
                            f"auto-pull failed for {name}: "
                            f"{bag_info['pull_error']} — "
                            "bag is still in /bags volume, "
                            "retrieve with `tools/pull-bag.sh`",
                        )
            # Keep the dict around with its terminal state so the
            # next /api/event/state response can still surface the
            # final bag path — cleared by the next event_start.
    log_event("pipeline", "Pipeline stopped (autonomy torn down)")
    response = {"ok": True, "pipeline": "stopped", "via": "ros_action"}
    if bag_info is not None:
        response["bag"] = bag_info
    return response


@app.get("/api/pipeline/status")
def pipeline_status():
    """Report whether autonomy is currently active.

    Reflects the actual lifecycle state of `control_node` — the leaf
    consumer in the mode_manager bring-up order. If control_node is
    `active`, everything upstream of it (odometry_filter, cone_detection, slam,
    planning) is too. Checking control_node alone is one rclpy call
    instead of four; cached in ros_bridge for 500 ms so the 1 Hz
    telemetry tick doesn't storm the get_state service.

    Pre-#392 this checked `is_action_server_available()` instead,
    which stayed True after a `Stop Session` tear-down (the
    management trio remains active) and caused the UI's PIPELINE
    RUNNING indicator to stick green forever.
    """
    if not _ros_bridge_available:
        return {"enabled": False, "reason": "ros_bridge unavailable"}
    return {
        "enabled": RosBridge.get().is_pipeline_active(),
    }


# === Vehicle ===

@app.get("/api/vehicle/state")
def vehicle_state():
    try:
        state = sim.get_vehicle_state()
    except Exception:
        return {}
    _ensure_home_pose_captured()
    return state

@app.get("/api/vehicle/pose")
def vehicle_pose():
    try:
        return sim.get_vehicle_pose()
    except Exception:
        return {}

@app.post("/api/vehicle/teleport", dependencies=[Depends(require_api_key)])
def vehicle_teleport(req: TeleportRequest):
    try:
        # Position-only — sim.teleport() defaults to qw=1 (identity quaternion)
        # which would snap the car to face East regardless of its current
        # heading. teleport_pos sends `simSetVehiclePose x y z` without the
        # quaternion, which the plugin treats as a position-only teleport
        # and preserves the actor's existing yaw across ResetVehicleState.
        sim.teleport_pos(req.x, req.y, req.z)
        return {"teleported": True}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)


# === Track Manager ===

# Path-traversal validators live in `track_validators.py` so the CI test
# suite can exercise them as pure functions without standing up the
# whole FastAPI app. Thin wrappers below convert `TrackNameError` into
# the `HTTPException(400)` shape the endpoints expect.
from track_validators import (
    TrackNameError,
    validate_track_name as _vt_name,
    validate_track_stem as _vt_stem,
    resolve_track_path as _rt_path,
)


def _validate_track_name(name: str) -> str:
    try:
        return _vt_name(name)
    except TrackNameError as e:
        raise HTTPException(status_code=400, detail=str(e))


def _validate_track_stem(stem: str) -> str:
    try:
        return _vt_stem(stem)
    except TrackNameError as e:
        raise HTTPException(status_code=400, detail=str(e))


def _resolve_track_path(name: str) -> str:
    try:
        return _rt_path(name, TRACKS_DIR)
    except TrackNameError as e:
        raise HTTPException(status_code=400, detail=str(e))


def parse_track_csv(filepath):
    cones = {"blue": [], "yellow": [], "big_orange": [], "small_orange": []}
    try:
        with open(filepath, "r") as f:
            for line in f:
                parts = line.strip().split(",")
                if len(parts) < 3:
                    continue
                cone_type = parts[0].strip()
                x, y = float(parts[1]), float(parts[2])
                if cone_type in cones:
                    cones[cone_type].append((x, y))
    except Exception:
        pass
    return cones

@app.get("/api/track/list")
def track_list():
    tracks = []
    for f in sorted(glob.glob(os.path.join(TRACKS_DIR, "*.csv"))):
        name = os.path.basename(f)
        cones = parse_track_csv(f)
        total = sum(len(v) for v in cones.values())
        tracks.append({
            "name": name, "path": os.path.abspath(f), "cones": total,
            "blue": len(cones["blue"]), "yellow": len(cones["yellow"]),
            "orange": len(cones["big_orange"]) + len(cones["small_orange"]),
            "builtin": name in BUILTIN_TRACKS,
            "event_type": BUILTIN_TRACKS.get(name),
        })
    return tracks

@app.get("/api/track/{name}/preview")
def track_preview(name: str):
    name = _validate_track_name(name)
    filepath = _resolve_track_path(name)
    if not os.path.exists(filepath):
        return JSONResponse({"error": "Track not found"}, status_code=404)

    cones = parse_track_csv(filepath)
    plot = generate_track_plot(cones)
    if plot:
        return {"image": plot}
    return JSONResponse({"error": "Failed to generate plot"}, status_code=500)

@app.post("/api/track/{name}/load", dependencies=[Depends(require_api_key)])
def track_load(name: str):
    global current_event, current_track
    name = _validate_track_name(name)
    filepath = _resolve_track_path(name)
    if not os.path.exists(filepath):
        return JSONResponse({"error": "Track not found"}, status_code=404)
    # `ue5_path` is sent to UE5's loadTrack RPC; the validated name
    # (basename only, no `..`, no slashes) keeps this scoped to the
    # mounted tracks volume on the sim side too.
    ue5_path = os.path.join(UE5_TRACKS_DIR, name)
    event_type = BUILTIN_TRACKS.get(name)
    with _state_lock:
        result = sim.load_track(ue5_path)
        if event_type:
            sim.set_event(event_type)
            current_event = event_type
            _save_state({"event": current_event})
        # #465 — record the basename so a subsequent record_bag start
        # composes a name that says which track this lap was on.
        # Cleared only on track-load failure; a fresh sim boot leaves
        # it None and the bag-name builder falls back to "no-track".
        current_track = name
        log_event("track_load", f"Loaded {name}" + (f" (event: {event_type})" if event_type else ""))

    # Tear down any running autonomy outside the state lock — a stale
    # SLAM map from the previous track is invalid for the new track.
    # Idempotent post-#379 (no-op if autonomy isn't active). Skipped
    # when ros_bridge isn't available — the action chain is the only
    # path post-#381.
    if _ros_bridge_available:
        try:
            RosBridge.get().stop_mission()
        except Exception as ex:
            log_event("track_load",
                      f"autonomy teardown failed (track loaded anyway): {ex}")
    return {"result": result, "track": name, "event_type": event_type}


@app.post("/api/vehicle/capture_home", dependencies=[Depends(require_api_key)])
def capture_home():
    """Capture current vehicle pose as the home/reset position."""
    global home_pose, _home_pose_captured
    pose = sim.get_vehicle_pose()
    if not pose:
        return JSONResponse({"ok": False, "error": "sim not connected"}, status_code=503)
    new_home = {
        "x": pose.get("x", 0.0),
        "y": pose.get("y", 0.0),
        "z": 0.3,
        "qw": pose.get("qw", 1.0),
        "qx": pose.get("qx", 0.0),
        "qy": pose.get("qy", 0.0),
        "qz": pose.get("qz", 0.0),
    }
    # Atomic publish under `_home_lock` (#327 B5). Pre-fix this did
    # the dict assignment and flag write lock-free, so a concurrent
    # `_ensure_home_pose_captured` could see the dict half-updated.
    with _home_lock:
        home_pose = new_home
        _home_pose_captured = True
        snapshot = dict(home_pose)
    return {"ok": True, "home_pose": snapshot}

@app.delete("/api/track/{name}", dependencies=[Depends(require_api_key)])
def track_delete(name: str):
    # Pre-#327 (B4) this had its own ad-hoc `if "/" in name or "\\" in
    # name or ".." in name` check that the other two endpoints lacked
    # — replaced with the shared `_validate_track_name` helper for
    # consistency.
    name = _validate_track_name(name)
    if name in BUILTIN_TRACKS:
        return JSONResponse({"error": "Cannot delete built-in track"}, status_code=400)
    filepath = _resolve_track_path(name)
    if os.path.exists(filepath):
        os.remove(filepath)
        return {"deleted": name}
    return JSONResponse({"error": "Not found"}, status_code=404)

@app.post("/api/track/generate", dependencies=[Depends(require_api_key)])
def track_generate(params: TrackGenerate):
    try:
        # `sys.path` insertion now happens once at module load
        # (#327 B9). Imports here are still local because the
        # third-party package only resolves after that insert and
        # we want the failure mode (missing dependency) to surface
        # as a 500 from this endpoint rather than a hard import
        # error at app startup that takes the whole API down.
        from track_generator import TrackGenerator
        from utils import Mode, SimType

        # Pre-#327 (B4) `params.name` was used directly as the
        # destination filename stem — `name="../../etc/passwd"`
        # would have written to TRACKS_DIR/../../etc/passwd.csv.
        # `_validate_track_stem` rejects anything outside
        # `[A-Za-z0-9._-]`. The auto-generated default is
        # always-safe.
        name_base = _validate_track_stem(params.name.strip()) if params.name.strip() else f"track_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        # output_yaml() builds its path as: os.path.realpath(os.path.dirname(__file__)) + output_location
        # __file__ is inside TRACK_GEN_PATH (read-only mount), so we use a traversal to
        # redirect output into a writable tmpdir: TRACK_GEN_PATH + /../../tmp/xxx = /tmp/xxx
        import tempfile
        tmp_dir = tempfile.mkdtemp()
        rel_to_gen = os.path.relpath(tmp_dir, TRACK_GEN_PATH)
        output_location = "/" + rel_to_gen  # e.g. "/../../tmp/tmpXXXXXX"

        # `os.chdir` is process-global. Two concurrent track-generate
        # calls would race on CWD; an unrelated handler running in the
        # same process between chdir-in and chdir-out would also see
        # the wrong CWD. Serialize with a dedicated lock and restore
        # CWD in finally so an exception inside `create_track()`
        # doesn't leave the whole backend stuck in TRACK_GEN_PATH.
        with _gen_lock:
            orig_dir = os.getcwd()
            os.chdir(TRACK_GEN_PATH)
            try:
                gen = TrackGenerator(
                    n_points=params.n_points, n_regions=params.n_regions,
                    min_bound=10., max_bound=float(params.max_bound),
                    mode=Mode.RANDOM, plot_track=False, visualise_voronoi=False,
                    create_output_file=True, output_location=output_location,
                    sim_type=SimType.FSDS
                )
                gen.create_track()
            finally:
                os.chdir(orig_dir)

        gen_file = os.path.join(tmp_dir, "random_track.csv")

        if os.path.exists(gen_file):
            os.makedirs(TRACKS_DIR, exist_ok=True)
            dest = os.path.join(TRACKS_DIR, f"{name_base}.csv")
            shutil.move(gen_file, dest)
            try:
                shutil.rmtree(tmp_dir)
            except Exception:
                pass

            cones = parse_track_csv(dest)
            total = sum(len(v) for v in cones.values())
            log_event("track_generate", f"Generated {name_base}.csv ({total} cones)")
            return {"name": f"{name_base}.csv", "cones": total}

        return JSONResponse({"error": "Generation failed"}, status_code=500)
    except ImportError as e:
        return JSONResponse({"error": f"Track generator not found: {e}"}, status_code=500)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)


# === Scoring ===

@app.get("/api/scoring/summary")
def scoring_summary(
    t_best: Optional[float] = Query(None),
    uss: bool = Query(False),
):
    ref = sim.get_referee_state()
    if not ref:
        return {"error": "No referee data"}

    event = ref.get("event", "unknown")
    lap_times = ref.get("lap_times", [])
    doo = ref.get("doo_counter", 0)
    oc = ref.get("oc_counter", 0)

    return compute_scoring(event, lap_times, doo, oc, t_best, uss=uss)


# === Session Log ===

def log_event(event_type: str, message: str):
    session_log.append({
        "timestamp": datetime.now().isoformat(),
        "type": event_type,
        "message": message,
    })

@app.get("/api/session/log")
def get_session_log():
    # Convert deque → list for JSON serialisation. FastAPI's default
    # encoder handles `deque` via the iterable path but jsonschema /
    # response_model introspection relies on `list`, so we cast
    # explicitly. The frontend's `Array.isArray` shape guard
    # (#326 F6) needs an actual array on the wire too.
    return list(session_log)

@app.get("/api/session/export")
def export_session_log():
    """Export session log as JSON download."""
    return Response(
        content=json.dumps(list(session_log), indent=2),
        media_type="application/json",
        headers={"Content-Disposition": f"attachment; filename=session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"}
    )


# === WebSocket Telemetry ===

# WS loop tuning (#327 B8). Pre-#327 the loop ran a flat 200 ms tick
# (5 Hz) and used a bare `except Exception` that, combined with B2's
# 3 s connect-timeout, made a downed sim into 3+ Hz of full-fat
# connect attempts × N WS clients. Adaptive: tick fast when the sim
# is up (need every frame for the dashboard), slow way down when
# it's known down (B2's cache short-circuits, no point telling the
# UI 5 times a second that the sim is still down).
_WS_NORMAL_TICK_S = 0.2
_WS_DISCONNECT_TICK_S = 2.0
# `_WS_SEND_TIMEOUT_S` puts a bound on how long we wait for a single
# `send_json` to land. Pre-#327 a half-open client (laptop sleeps,
# wifi hangs) was only detected when the OS eventually noticed the
# dead TCP — could take minutes. The wait_for timeout gives us an
# active-detection window without an explicit ping/pong protocol.
_WS_SEND_TIMEOUT_S = 5.0


async def _ws_safe_send(websocket: WebSocket, data: dict) -> bool:
    """Send `data` with a timeout. Returns True on success, False if
    the peer is gone or stalled (caller should `break` the loop)."""
    try:
        await asyncio.wait_for(
            websocket.send_json(data),
            timeout=_WS_SEND_TIMEOUT_S,
        )
        return True
    except (asyncio.TimeoutError, RuntimeError, ConnectionError, WebSocketDisconnect):
        return False


@app.websocket("/ws/telemetry")
async def telemetry_ws(websocket: WebSocket, api_key: Optional[str] = Query(default=None)):
    # WebSocket handshakes from browsers can't carry custom headers, so
    # the API key (if the server is configured with one) arrives as a
    # query parameter. Reject before accept so the handshake never
    # completes for an unauthenticated caller. The literal 1008 is the
    # "policy violation" WS close code — hardcoded here rather than
    # using `status.WS_1008_POLICY_VIOLATION` because the loop below
    # reuses the name `status` for `sim.get_status()`, which would
    # shadow the fastapi `status` module under Python's function-scope
    # rule and UnboundLocalError this line.
    if not ws_api_key_ok(api_key):
        await websocket.close(code=1008)
        return
    await websocket.accept()
    try:
        while True:
            tick = _WS_NORMAL_TICK_S

            # Connection probe. With #342's caching this is sub-µs in
            # the common case (recent successful command) and capped
            # at one connect-timeout-attempt per 5 s when known down.
            connected = await asyncio.to_thread(sim.is_connected)

            if not connected:
                # Sim known down. Send the beacon (frontend's shape
                # gate at #326 F4 drops it) and loop slowly. This is
                # the path that pre-#327 hammered the connect timeout
                # at 3+ Hz × N clients.
                if not await _ws_safe_send(websocket, {"error": "sim_disconnected"}):
                    break
                tick = _WS_DISCONNECT_TICK_S
            else:
                try:
                    # Gather state from sim. The three calls go through
                    # SimConnection's threading.Lock, so a slow compound
                    # operation in another worker (event_start does ~5
                    # sequential RPCs while holding the lock) would
                    # freeze the event loop here for hundreds of ms —
                    # visible in the UI as telemetry "skipping". Run on
                    # the thread pool so the loop stays responsive
                    # while we block on the wire.
                    vehicle = await asyncio.to_thread(sim.get_vehicle_state)
                    ref = await asyncio.to_thread(sim.get_referee_state)
                    sim_status = await asyncio.to_thread(sim.get_status)

                    # Snapshot the shared mutables under their lock so
                    # a mid-toggle RES (or mid-capture home pose)
                    # doesn't leave us reading an inconsistent state
                    # into the WS frame (#327 B5).
                    with _state_lock:
                        res_active_snap = res_active
                        current_event_snap = current_event
                        # #465 — bag-record state. WS pushes it on
                        # every 1 Hz tick so the UI can swap its
                        # badge between "recording" / "stopped" /
                        # "failed" as the lifecycle moves.
                        if _active_recording:
                            bag_name_snap = _active_recording.get("name")
                            bag_state_snap = _active_recording.get("state", "none")
                            # #498 — once auto_pull_and_clean has run,
                            # host_path is the absolute container-side
                            # path (bind-mounted to host ./bags/<name>/).
                            # Empty/None until the pull completes.
                            bag_host_path_snap = _active_recording.get("host_path") or None
                        else:
                            bag_name_snap = None
                            bag_state_snap = "none"
                            bag_host_path_snap = None

                    data = {
                        "speed": vehicle.get("speed", 0),
                        "rpm": vehicle.get("rpm", 0),
                        "gear": vehicle.get("gear", 0),
                        "x": vehicle.get("x", 0),
                        "y": vehicle.get("y", 0),
                        "z": vehicle.get("z", 0),
                        "throttle": vehicle.get("controls", {}).get("throttle", 0),
                        "steering": vehicle.get("controls", {}).get("steering", 0),
                        "brake": vehicle.get("controls", {}).get("brake", 0),
                        # Regen telemetry (motor-side). regen_torque/power
                        # reflect what the sim is currently absorbing;
                        # regen_avail_torque is the cap at the current
                        # ω_motor (motor-peak or cell-power-limited,
                        # whichever binds); regen_max_*_limit are the
                        # hardware ceilings from settings.json.
                        "regen_torque": vehicle.get("regen_torque", 0),
                        "regen_power": vehicle.get("regen_power", 0),
                        "regen_avail_torque": vehicle.get("regen_avail_torque", 0),
                        "regen_max_torque": vehicle.get("regen_max_torque", 0),
                        "regen_max_power": vehicle.get("regen_max_power", 0),
                        "doo": ref.get("doo_counter", 0),
                        "oc": ref.get("oc_counter", 0),
                        "laps": ref.get("laps", 0),
                        "required_laps": ref.get("required_laps", 0),
                        "finished": ref.get("finished", False),
                        "event": current_event_snap,
                        "fps": sim_status.get("fps", 0),
                        "paused": sim_status.get("paused", False),
                        "res_active": res_active_snap,
                        # Reflects whether control_node is in lifecycle
                        # state `active` — the accurate "autonomy is
                        # running" signal. Cached in ros_bridge for
                        # 500 ms so this 1 Hz tick doesn't storm
                        # /control_node/get_state. Pre-#392 this used
                        # action-server reachability, which stayed
                        # true after Stop Session and stuck the UI
                        # indicator green.
                        "pipeline_enabled": (
                            _ros_bridge_available
                            and RosBridge.get().is_pipeline_active()
                        ),
                        # #465 — bag-record live state for the UI's
                        # "● RECORDING" badge.
                        "bag_name": bag_name_snap,
                        "bag_state": bag_state_snap,
                        # #498 — host-side path after auto-pull. UI
                        # uses this to flip the post-stop badge from
                        # "saved in container" to "saved to host".
                        "bag_host_path": bag_host_path_snap,
                    }

                    if not await _ws_safe_send(websocket, data):
                        break
                except WebSocketDisconnect:
                    # Re-raise to the outer try so the cleanup is
                    # consistent with a peer-initiated disconnect.
                    raise
                except Exception:
                    # RPC error during a tick where is_connected said
                    # we were up — race: sim went down between
                    # is_connected and the actual call. Send a beacon
                    # and back off to the disconnected tick rate so
                    # we're not spinning while the cache catches up.
                    if not await _ws_safe_send(websocket, {"error": "sim_error"}):
                        break
                    tick = _WS_DISCONNECT_TICK_S

            await asyncio.sleep(tick)
    except WebSocketDisconnect:
        pass


# === Track Plot Helper ===

def generate_track_plot(cones):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(1, 1, figsize=(8, 6))
        fig.patch.set_facecolor("#1a1a1a")
        ax.set_facecolor("#111111")

        if cones.get("blue"):
            bx, by = zip(*cones["blue"])
            ax.plot(bx, by, "o", color="#4488ff", markersize=4, label=f'Blue ({len(cones["blue"])})')
        if cones.get("yellow"):
            yx, yy = zip(*cones["yellow"])
            ax.plot(yx, yy, "o", color="#ffb81c", markersize=4, label=f'Yellow ({len(cones["yellow"])})')
        if cones.get("big_orange"):
            ox, oy = zip(*cones["big_orange"])
            ax.plot(ox, oy, "^", color="#ff6600", markersize=10, label=f'Orange ({len(cones["big_orange"])})')
        if cones.get("small_orange"):
            sx, sy = zip(*cones["small_orange"])
            ax.plot(sx, sy, "v", color="#ff8800", markersize=7)

        ax.set_aspect("equal")
        ax.set_xlabel("X (meters)", color="#888")
        ax.set_ylabel("Y (meters)", color="#888")
        ax.tick_params(colors="#888")
        ax.legend(loc="upper right", facecolor="#222", edgecolor="#333", labelcolor="#ccc", fontsize=9)
        ax.grid(True, alpha=0.15, color="#ffb81c")
        for spine in ax.spines.values():
            spine.set_color("#333")

        buf = io.BytesIO()
        fig.savefig(buf, format="png", bbox_inches="tight", dpi=120, facecolor=fig.get_facecolor())
        plt.close(fig)
        buf.seek(0)
        return base64.b64encode(buf.read()).decode()
    except Exception as e:
        print(f"Plot error: {e}")
        return None


# === Startup / shutdown ===

# Whether the rclpy-backed RosBridge initialised successfully. The
# backend tolerates a missing rclpy install (e.g. when running unit
# tests outside Docker) by gracefully falling back to the legacy
# flag-file-driven /api/pipeline/start path. In production
# (mission_control_backend container, ROS 2 base image) this should
# always be True.
_ros_bridge_available = False


@app.on_event("startup")
def startup():
    global _ros_bridge_available
    print(f"IFSSIM Mission Control | http://localhost:8000")
    print(f"Sim: {SIM_HOST}:{SIM_PORT}")
    print(f"Tracks: {TRACKS_DIR}")
    print(f"Docs: http://localhost:8000/docs")

    # Spin up the rclpy bridge so /api/pipeline/start can call
    # sim_supervisor_node.StartMission. Failure is non-fatal — we
    # log and fall back to the flag-file path in pipeline_start().
    try:
        RosBridge.start()
        _ros_bridge_available = True
        print("ros_bridge: ready (StartMission action client active)")
    except Exception as ex:
        _ros_bridge_available = False
        print(f"ros_bridge: NOT available ({ex.__class__.__name__}: {ex}); "
              "/api/pipeline/start will use the legacy flag-file path")


@app.on_event("shutdown")
def shutdown():
    if _ros_bridge_available:
        try:
            RosBridge.shutdown()
        except Exception as ex:
            print(f"ros_bridge shutdown error (ignored): {ex}")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
