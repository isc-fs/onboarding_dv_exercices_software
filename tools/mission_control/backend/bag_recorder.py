"""Bag-recording client for the Mission Control session UX (#465).

Calls /bag_recorder/start + /bag_recorder/stop on the dv_pipeline_stack
DDS graph. The actual `ros2 bag record` subprocess lives inside the
dv_pipeline_stack container (see pipeline/bag_recorder_node/) where
it shares the SHM-tuned Fast DDS context with the publishers — that's
the only way to get full-fidelity 10 Hz LiDAR + camera capture, since
mc_backend is forced to UDPv4-only for its StartMission action client.

## Wire-format contract

  compose_bag_name(event_type, track) → str   (kept here — name
      synthesis is a backend concern, the ROS node accepts whatever
      string the backend hands it)

  request_start(ros_bridge, bag_name) → dict:
      ok          — bool
      state       — "recording" | "failed"
      name        — bag_name (echoed)
      path        — absolute host path of final bag dir
      error       — diagnostic on ok=false

  request_stop(ros_bridge) → dict:
      ok          — bool
      state       — "stopped" | "failed" | "none"
      path        — absolute host path of finalised bag dir
      error       — diagnostic on ok=false

Returns plain dicts (not ROS response objects) so main.py's lifecycle
code can store + serialise them without re-importing rclpy. The
ros_bridge handle is the existing one from `ros_bridge.py` — this
module just borrows its rclpy.Node to host two service clients.
"""
from __future__ import annotations

import logging
import os
import re
import tarfile
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

_LOG = logging.getLogger(__name__)


# Name sanitisation lives here (not in the ROS node) because it's a
# pure function and the mc_backend is the only producer of bag names.
# Keeping it on this side means the test suite can drive it without
# spinning up rclpy.
_NAME_BAD = re.compile(r"[^A-Za-z0-9_-]+")


def _sanitize(s: str) -> str:
    s = Path(s).stem
    return _NAME_BAD.sub("_", s).strip("_")[:48]


def compose_bag_name(
    event_type: str,
    track: Optional[str],
    now: Optional[datetime] = None,
) -> str:
    """Return a filesystem-safe bag dir name.

    Shape: `<event>_<track>_<YYYYMMDD_HHMMSS>`. Either segment may be
    absent (we fall back to "unknown" / "no-track"). The timestamp is
    always present so consecutive recordings can't collide.
    """
    when = (now or datetime.now(timezone.utc)).strftime("%Y%m%d_%H%M%S")
    event = _sanitize(event_type) or "unknown"
    trk = _sanitize(track) if track else "no-track"
    return f"{event}_{trk}_{when}"


# Service client cache — created lazily on the FIRST request_start /
# request_stop call so the import path stays cheap when ROS isn't
# present (CI, unit tests). Re-used across calls because creating a
# rclpy service client involves DDS discovery.
_clients_lock = threading.Lock()
_start_client = None
_stop_client = None
_start_srv_type = None
_stop_srv_type = None


def _ensure_clients(ros_bridge) -> None:
    """Lazy-init StartBag + StopBag service clients on the bridge's node."""
    global _start_client, _stop_client, _start_srv_type, _stop_srv_type
    with _clients_lock:
        if _start_client is not None and _stop_client is not None:
            return
        from dv_msgs.srv import StartBag, StopBag
        _start_srv_type = StartBag
        _stop_srv_type = StopBag
        _start_client = ros_bridge._node.create_client(
            StartBag, "/bag_recorder/start",
        )
        _stop_client = ros_bridge._node.create_client(
            StopBag, "/bag_recorder/stop",
        )


def _wait_service(client, name: str, timeout_s: float) -> bool:
    """Block until the service server is reachable, or timeout."""
    if not client.wait_for_service(timeout_sec=timeout_s):
        _LOG.warning(
            "bag_recorder: %s server not reachable within %.1fs", name, timeout_s,
        )
        return False
    return True


def _call_sync(client, request, timeout_s: float):
    """Send a service request synchronously via rclpy's spin executor.

    ros_bridge spins its node on a dedicated thread, so we can just
    fire-and-wait on the future. Returns the response or None on
    timeout / failure.
    """
    future = client.call_async(request)
    # ros_bridge's executor will tick this future on its own thread.
    # We block here for up to timeout_s.
    deadline = threading.Event()

    def _on_done(_fut):
        deadline.set()

    future.add_done_callback(_on_done)
    if not deadline.wait(timeout_s):
        return None
    if future.exception() is not None:
        _LOG.warning("bag_recorder: service raised: %s", future.exception())
        return None
    return future.result()


def request_start(ros_bridge, bag_name: str, *, timeout_s: float = 5.0) -> dict:
    """Ask bag_recorder_node to start a recording.

    Returns a state dict (ok / state / name / path / error). On any
    failure to reach the service, ok=false with a diagnostic.
    """
    try:
        _ensure_clients(ros_bridge)
    except Exception as ex:
        return {
            "ok": False,
            "state": "failed",
            "name": bag_name,
            "path": "",
            "error": f"bag_recorder service client init failed: {ex}",
        }

    if not _wait_service(_start_client, "/bag_recorder/start", timeout_s):
        return {
            "ok": False,
            "state": "failed",
            "name": bag_name,
            "path": "",
            "error": (
                "/bag_recorder/start not reachable — is the "
                "dv_pipeline_stack container running and healthy?"
            ),
        }

    req = _start_srv_type.Request()
    req.bag_name = bag_name

    resp = _call_sync(_start_client, req, timeout_s)
    if resp is None:
        return {
            "ok": False,
            "state": "failed",
            "name": bag_name,
            "path": "",
            "error": f"/bag_recorder/start timed out after {timeout_s:.1f}s",
        }

    return {
        "ok": bool(resp.ok),
        "state": resp.state or ("recording" if resp.ok else "failed"),
        "name": bag_name,
        "path": resp.bag_path,
        "error": resp.error,
    }


def request_stop(ros_bridge, *, timeout_s: float = 15.0) -> dict:
    """Ask bag_recorder_node to stop the active recording.

    Idempotent: returns state="none" if nothing was recording. Timeout
    is intentionally generous (15 s default) because the server side
    has to SIGINT the recorder, wait for mcap to flush its chunk
    index, then move the staged dir into the bind-mounted output
    directory — the move can be a couple of seconds for a multi-GB
    bag on macOS virtiofs.
    """
    try:
        _ensure_clients(ros_bridge)
    except Exception as ex:
        return {
            "ok": False,
            "state": "failed",
            "path": "",
            "error": f"bag_recorder service client init failed: {ex}",
        }

    if not _wait_service(_stop_client, "/bag_recorder/stop", timeout_s=2.0):
        return {
            "ok": False,
            "state": "failed",
            "path": "",
            "error": "/bag_recorder/stop not reachable",
        }

    req = _stop_srv_type.Request()
    resp = _call_sync(_stop_client, req, timeout_s)
    if resp is None:
        return {
            "ok": False,
            "state": "failed",
            "path": "",
            "error": f"/bag_recorder/stop timed out after {timeout_s:.1f}s",
        }

    return {
        "ok": bool(resp.ok),
        "state": resp.state or ("stopped" if resp.ok else "failed"),
        "path": resp.bag_path,
        "error": resp.error,
    }


def _reset_clients_for_test() -> None:
    """Test-only: drop the cached clients so a new ros_bridge mock is picked up."""
    global _start_client, _stop_client, _start_srv_type, _stop_srv_type
    with _clients_lock:
        _start_client = None
        _stop_client = None
        _start_srv_type = None
        _stop_srv_type = None


# ---------------------------------------------------------------------
# #498 — auto-pull a finalised bag from the dv_pipeline_stack volume
# onto the host filesystem.
# ---------------------------------------------------------------------
# Gated by the IFSSIM_BAG_AUTO_PULL env var. When set, the StopBag
# handler in main.py calls `auto_pull_and_clean(bag_name)` after a
# successful stop; we use the Python `docker` SDK to:
#   1. `container.get_archive(/bags/<name>)` → stream the bag as a tar
#      from dv_pipeline_stack, write it to a host-bind-mounted
#      destination (/host_bags/ → host ./bags/).
#   2. `container.exec_run("rm -rf /bags/<name>")` → clean the
#      volume-side copy.
#
# We picked the SDK over CLI subprocess for one practical reason: the
# Linux `docker cp` CLI inside mc_backend can't pass a Windows host
# path (`C:/Users/...`) because it parses at the first colon and
# treats `C` as a container name. The SDK uses the HTTP API directly,
# no argv parsing, no path translation involved on our end — we just
# write the tarball through the bind-mount.
#
# The bind-mount (/host_bags/) brings back a small bit of the
# virtiofs/9p slowness #490 retired, but only for the *write* of the
# finalised tarball at session-stop. The recording itself lands in
# the named volume on container ext4 (fast); this only kicks in once
# the bag is closed.
_DEFAULT_PULL_TIMEOUT_S = 120.0
# Host-side mount where mc_backend writes the pulled bag. Bind-mounted
# in docker-compose.yml to `./bags/`.
_HOST_BAGS_DIR = "/host_bags"


def _get_docker_client():
    """Lazy-construct and cache a docker SDK client.

    Cached because socket-discovery + initial handshake is a few ms;
    we keep one client per process. Probing connectivity here lets
    `is_auto_pull_ready` return a clean diagnostic without a partial
    pull attempt.
    """
    global _docker_client_cache
    try:
        return _docker_client_cache
    except NameError:
        pass
    try:
        import docker  # type: ignore[import]
        client = docker.from_env(timeout=3)
        # Probe — raises on broken socket / daemon down / permission.
        client.ping()
    except Exception as ex:  # noqa: BLE001 — surfaces any SDK failure
        _LOG.warning(
            "bag_recorder: docker SDK unavailable (%s) — auto-pull disabled",
            ex,
        )
        client = None
    _docker_client_cache = client
    return client


def is_auto_pull_enabled() -> bool:
    """Read the env-gate at call-time so a `docker compose restart`
    with the flag flipped is enough to change behaviour."""
    raw = os.environ.get("IFSSIM_BAG_AUTO_PULL", "1").strip().lower()
    return raw in ("1", "true", "yes", "on")


def _ensure_host_dir() -> Optional[str]:
    """Ensure the host-bags landing directory exists from mc_backend's
    perspective (bind-mounted to the host's ./bags/). Returns the path
    on success, None if the bind-mount isn't where we expect it.
    """
    p = Path(_HOST_BAGS_DIR)
    try:
        p.mkdir(parents=True, exist_ok=True)
    except OSError as ex:
        _LOG.warning("bag_recorder: cannot create %s: %s", _HOST_BAGS_DIR, ex)
        return None
    return str(p)


def auto_pull_and_clean(
    bag_name: str,
    *,
    timeout_s: float = _DEFAULT_PULL_TIMEOUT_S,
) -> dict:
    """Move a finalised bag from the dv_pipeline_stack volume to the
    host filesystem, then delete the volume-side copy.

    Returns a dict with:
        ok          — bool. False on any failure (transfer, rm, env).
        bag_name    — echoed.
        host_path   — final container-side path to the bag (which is
                      `/host_bags/<bag_name>`, bind-mounted to the
                      host's `./bags/<bag_name>`). Empty if not pulled.
        error       — diagnostic on ok=false. Empty on success.

    Failure semantics: on transfer failure we DO NOT delete the
    volume-side copy (the user can recover with `tools/pull-bag.sh`).
    On rm failure we keep ok=true and surface a warning in `error` —
    the bag is safe on the host, the volume orphan is an annoyance,
    not a data-loss risk.
    """
    out: dict = {
        "ok": False,
        "bag_name": bag_name,
        "host_path": "",
        "error": "",
    }

    if not is_auto_pull_enabled():
        out["error"] = "IFSSIM_BAG_AUTO_PULL disabled"
        return out
    if not bag_name:
        out["error"] = "empty bag_name"
        return out

    # Defence in depth: reject suspicious bag names that could escape
    # /host_bags via traversal. Recorder's compose_bag_name sanitises
    # upstream, but anything we untar runs in mc_backend's filesystem.
    if "/" in bag_name or ".." in bag_name or bag_name.startswith("-"):
        out["error"] = f"refusing to pull bag with suspicious name: {bag_name!r}"
        return out

    client = _get_docker_client()
    if client is None:
        out["error"] = "docker SDK / docker.sock unavailable from mc_backend"
        return out

    container_name = os.environ.get(
        "DV_PIPELINE_STACK_CONTAINER", "ifssim-dv_pipeline_stack-1",
    ).strip()
    host_dir = _ensure_host_dir()
    if host_dir is None:
        out["error"] = (
            f"host-bags bind-mount {_HOST_BAGS_DIR!r} not present — "
            "is mc_backend missing the `./bags:/host_bags` mount?"
        )
        return out

    try:
        container = client.containers.get(container_name)
    except Exception as ex:  # noqa: BLE001
        out["error"] = f"container {container_name!r} not found: {ex}"
        return out

    # Stream the bag as a tarball from the source container. get_archive
    # returns (bits_iter, stat_dict) where bits_iter yields tar chunks
    # and stat_dict has a `size` field for diagnostics.
    src_path = f"/bags/{bag_name}"
    try:
        bits, _stat = container.get_archive(src_path)
    except Exception as ex:  # noqa: BLE001
        out["error"] = f"get_archive({src_path!r}) failed: {ex}"
        return out

    # Untar directly into the host-bind-mount. The tarball's root
    # entry is the source basename (i.e. `<bag_name>/`), so extracting
    # to /host_bags/ recreates /host_bags/<bag_name>/... — same
    # ergonomics as the `docker cp` parent-dir convention.
    #
    # Stream the tarball to a temp file on disk first, then untar from
    # there. Prior implementation buffered the whole archive in a
    # BytesIO before extracting; that allocates `len(bag)` bytes in
    # mc_backend's heap and tripped mem_limit: 512m on the container
    # for multi-hundred-MB bags. The container got SIGKILL'd mid-stop,
    # nginx surfaced a 502, and the volume-side cleanup never ran.
    # Streaming to disk caps live RAM at the chunk size (~256 KB).
    import tempfile
    tmp = tempfile.NamedTemporaryFile(
        prefix=f"bag_pull_{bag_name}_", suffix=".tar",
        dir=host_dir, delete=False,
    )
    try:
        for chunk in bits:
            tmp.write(chunk)
        tmp.flush()
        tmp.close()
    except Exception as ex:  # noqa: BLE001
        out["error"] = f"streaming tarball to temp file failed: {ex}"
        try:
            os.unlink(tmp.name)
        except OSError:
            pass
        return out

    try:
        with tarfile.open(name=tmp.name, mode="r") as tar:
            # `r|` is a streaming-read mode that's safe against large
            # archives. We don't try to validate every member's path
            # (`bag_name` is already sanitised) but we do refuse any
            # entry whose normalised path would escape `host_dir`.
            for member in tar:
                target = os.path.normpath(os.path.join(host_dir, member.name))
                if not (target == host_dir or target.startswith(host_dir + os.sep)):
                    out["error"] = (
                        f"tarball contains suspicious member path: {member.name!r} "
                        f"(would land outside {host_dir!r})"
                    )
                    return out
                # `filter="data"` is the Python 3.12+ safe-default
                # extraction policy: strips ownership / permission
                # bits beyond rw, blocks special files, blocks
                # absolute paths and `..` traversal at the tar level.
                # Python 3.14 will require this flag; setting it
                # explicitly also silences the deprecation warning
                # on 3.12 / 3.13. We additionally reject suspicious
                # members above as a belt-and-braces precaution.
                tar.extract(member, path=host_dir, filter="data")
    except Exception as ex:  # noqa: BLE001
        out["error"] = f"tar extract to {host_dir} failed: {ex}"
        try:
            os.unlink(tmp.name)
        except OSError:
            pass
        return out

    # Remove the temp tarball — we've extracted everything we need.
    try:
        os.unlink(tmp.name)
    except OSError as ex:
        _LOG.warning("bag_recorder: temp tarball cleanup failed: %s", ex)

    out["ok"] = True
    out["host_path"] = f"{host_dir}/{bag_name}"

    # Best-effort clean of the volume-side copy. Even on failure here,
    # the bag is safely on the host, so we keep ok=true and just
    # surface a warning.
    try:
        rc, output = container.exec_run(
            ["rm", "-rf", f"/bags/{bag_name}"],
            demux=False, stdout=True, stderr=True,
        )
        if rc != 0:
            text = (output or b"").decode(errors="replace").strip()[:200]
            out["error"] = (
                f"transfer ok, but cleanup failed (rc={rc}): {text}; "
                "bag is on the host, volume has an orphan."
            )
    except Exception as ex:  # noqa: BLE001
        out["error"] = (
            f"transfer ok, but cleanup raised: {ex}; "
            "bag is on the host, volume has an orphan."
        )

    return out


def _reset_docker_probe_cache_for_test() -> None:
    """Test-only: drop the cached docker SDK client so a new
    monkeypatched docker module is picked up."""
    global _docker_client_cache
    try:
        del _docker_client_cache
    except NameError:
        pass
