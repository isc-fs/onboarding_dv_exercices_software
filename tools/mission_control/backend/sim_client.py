"""
Sim client wrapper for the Mission Control backend.
Uses a persistent TCP connection (reconnects on failure) to avoid flooding
UE5 with a new connection for every API call.
All methods are safe to call when the sim is disconnected.
"""

import sys
import os
import json
import socket
import threading
import time
from typing import Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "python"))

from ifssim import IFSSIMClient


# Connection-state cache TTLs (#327 B2). Pre-#327 every `is_connected()`
# call did a real `ping` round-trip, and `_require_connected()` was
# called at the top of every command — so e.g. `release EBS` was two
# RPCs (ping + releaseEbs), the WS telemetry tick added a ping per
# state read, and a downed sim made every poll wait the full 3 s
# connect timeout. The positive TTL is short because the sim can
# disappear at any time (UE5 crash, Stop in editor); the negative
# TTL is longer because we don't want to burn the full connect
# timeout × N WS clients per second when it's known down.
_CONNECTED_CACHE_TTL_S = 1.0
_DISCONNECTED_CACHE_TTL_S = 5.0


class SimConnection:
    """Persistent TCP connection to the IFSSIM RPC server."""

    # Default per-command socket timeout for the read leg. 5 s is
    # comfortable for the fast-path RPCs (ping, getCarState,
    # getRefereeState, RES toggles) but trips on long-tail commands
    # like loadTrack on a large CSV (#327 B11). Per-call override via
    # the `timeout` kwarg on `_cmd`.
    _DEFAULT_READ_TIMEOUT_S = 5.0

    def __init__(self, host: str = "127.0.0.1", port: int = 41451):
        self.host = host
        self.port = port
        self._sock: socket.socket | None = None
        self._rbuf = b""
        self._lock = threading.Lock()
        # Connection-state cache. `_last_seen_alive` is updated inside
        # `_cmd` on every successful round-trip; `_last_failure` is
        # updated when `_cmd` exhausts its retry. `is_connected()`
        # consults both before issuing a real ping. Reads are
        # intentionally lock-free — single-attribute reads are atomic
        # under the GIL and a stale read just means at worst one
        # extra ping, no correctness hazard.
        self._last_seen_alive: float = 0.0
        self._last_failure: float = 0.0
        # `_known_dead` starts True so the very first `is_connected()`
        # call performs a real probe. After any successful command
        # this flips to False and stays there until a `_cmd` failure
        # flips it back.
        self._known_dead: bool = True

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def _connect(self) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0)
            s.connect((self.host, self.port))
            # `_cmd` resets the timeout per-invocation (#327 B11), so
            # this initial value is just a sane default for any code
            # path that hits the socket before the first `_cmd` call
            # (currently none, but defensive).
            s.settimeout(self._DEFAULT_READ_TIMEOUT_S)
            self._sock = s
            self._rbuf = b""
            return True
        except Exception:
            self._sock = None
            return False

    def _disconnect(self):
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None
        self._rbuf = b""

    # ------------------------------------------------------------------
    # Core send/receive — the server keeps the connection alive and sends
    # exactly one newline-terminated line per command.
    # ------------------------------------------------------------------

    def _cmd(self, cmd: str, timeout: Optional[float] = None) -> str:
        """Send `cmd`, return the response line (or "" on failure).

        `timeout` overrides the per-command socket read timeout for
        long-tail RPCs (#327 B11). Default 5 s is fine for ping /
        getCarState / RES toggles; loadTrack on a large CSV needs
        more headroom. The override applies only to this invocation;
        the next `_cmd` call resets to the default.
        """
        with self._lock:
            read_timeout = timeout if timeout is not None else self._DEFAULT_READ_TIMEOUT_S
            for attempt in range(2):
                try:
                    if self._sock is None and not self._connect():
                        # Connect itself failed — cache the failure so
                        # the next `is_connected()` short-circuits.
                        self._known_dead = True
                        self._last_failure = time.monotonic()
                        return ""

                    self._sock.settimeout(read_timeout)
                    self._sock.sendall((cmd + "\n").encode())

                    # Read until newline
                    while b"\n" not in self._rbuf:
                        chunk = self._sock.recv(65536)
                        if not chunk:
                            raise ConnectionError("socket closed by server")
                        self._rbuf += chunk

                    line, _, self._rbuf = self._rbuf.partition(b"\n")
                    # Successful round-trip — refresh the positive
                    # cache. `is_connected()` will skip the next ping
                    # if it's called within `_CONNECTED_CACHE_TTL_S`.
                    self._last_seen_alive = time.monotonic()
                    self._known_dead = False
                    return line.decode().strip()

                except Exception:
                    self._disconnect()
                    if attempt == 0:
                        continue
            # Both attempts failed — record the negative cache.
            self._known_dead = True
            self._last_failure = time.monotonic()
            return ""

    def _json_cmd(self, cmd: str, timeout: Optional[float] = None) -> dict:
        resp = self._cmd(cmd, timeout=timeout)
        if not resp:
            return {}
        try:
            return json.loads(resp)
        except Exception:
            return {"raw": resp}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def is_connected(self) -> bool:
        """Return whether the sim is currently reachable.

        Cached: a successful command in the last `_CONNECTED_CACHE_TTL_S`
        means we're still up (no ping needed); a known-dead state
        within `_DISCONNECTED_CACHE_TTL_S` short-circuits to False
        (no connect-timeout wait). Falls through to a real `ping` only
        when the cache expires or the state has never been
        established. Pre-#327 every call here was a full RPC
        round-trip; with that pattern multiplied across `_require_connected`
        and the 100 Hz WS telemetry loop, a downed sim was ~10
        connect-timeout-attempts/sec per client.
        """
        now = time.monotonic()
        # Positive cache hit — recent successful command implies up.
        if not self._known_dead and (now - self._last_seen_alive) < _CONNECTED_CACHE_TTL_S:
            return True
        # Negative cache hit — known dead and we tried recently. Don't
        # burn another 3 s connect timeout for this caller.
        if self._known_dead and (now - self._last_failure) < _DISCONNECTED_CACHE_TTL_S:
            return False
        # Cache stale — do a real probe. `_cmd` updates the cache on
        # both success and failure paths.
        return self._cmd("ping") == "true"

    def get_status(self) -> dict:
        result = self._json_cmd("getSimStatus")
        return {
            "map": result.get("map", "unknown"),
            "fps": result.get("fps", 0),
            "paused": result.get("paused", False),
            "api_control": result.get("api_control", False),
        }

    def _require_connected(self):
        """Raise RuntimeError if the sim is not reachable."""
        if not self.is_connected():
            raise RuntimeError("Simulator not connected")

    def pause(self):
        self._require_connected()
        self._cmd("simPause")

    def resume(self):
        self._require_connected()
        self._cmd("simResume")

    def is_paused(self) -> bool:
        return self._cmd("simIsPaused") == "true"

    def set_event(self, event_type: str, num_laps: int = 10) -> dict:
        self._require_connected()
        return self._json_cmd(f"setEventType {event_type} {num_laps}")

    def get_referee_state(self) -> dict:
        return self._json_cmd("getRefereeState")

    def res_activate(self):
        self._require_connected()
        # EBS / RES = the pneumatic handbrake on the real car. The older
        # `setCarControls 0 0 1 + disableApiControl` path was silently
        # undone every tick by UE5's keyboard axis-input system — the
        # "brake" axis reads 0 by default and overwrote the RPC-set
        # CurrentControls.Brake, leaving the car coasting on drag alone.
        # activateEbs latches the handbrake and locks all input channels.
        self._cmd("activateEbs")

    def res_release(self):
        self._require_connected()
        self._cmd("releaseEbs")

    def get_vehicle_state(self) -> dict:
        state = self._json_cmd("getCarState")
        controls = self._json_cmd("getCarControls")
        state["controls"] = controls
        return state

    def get_vehicle_pose(self) -> dict:
        return self._json_cmd("simGetVehiclePose")

    def get_start_gate_pose(self) -> dict | None:
        """Return the start-gate pose stored by the last loadTrack call,
        in ENU coordinates.

        The plugin's loadTrack handler computes the start-gate position
        via ComputeStartGatePose, teleports the car to it, and stores
        the result in `LastStartGateLoc_UE` / `LastStartGateRot_UE`.
        This RPC reads those stored values back (converted to ENU), so
        callers can ask "where IS the start gate?" without re-running
        the spawner. Returns the same canonical pose loadTrack aligned
        the car to — which is what /api/sim/reset wants when restoring
        the car to its "fresh load" pose.

        Returns None if no track has been loaded since the sim came up
        (the plugin's `bHasStartGate` flag is false). Callers should
        fall back to a captured `home_pose` or refuse the reset in
        that case.
        """
        res = self._json_cmd("getStartGatePose")
        if not isinstance(res, dict) or "error" in res or "x" not in res:
            return None
        return res

    def teleport(self, x: float, y: float, z: float,
                 qw: float = 1.0, qx: float = 0.0,
                 qy: float = 0.0, qz: float = 0.0):
        self._cmd(f"simSetVehiclePose {x} {y} {z} {qw} {qx} {qy} {qz}")

    def teleport_pos(self, x: float, y: float, z: float):
        """Teleport position only — orientation unchanged."""
        self._cmd(f"simSetVehiclePose {x} {y} {z}")

    def load_track(self, filepath: str) -> dict:
        # loadTrack can take well over 5 s on a large CSV — UE5
        # parses, spawns each cone actor, rebuilds nav. Pre-#327
        # the default 5 s timeout (#327 B11) tripped, the client
        # `_disconnect`ed and retried once, and the caller saw an
        # empty response and reported "load_track returned nothing".
        return self._json_cmd(f"loadTrack {filepath}", timeout=30.0)

    def get_settings(self) -> str:
        return self._cmd("getSettingsString")
