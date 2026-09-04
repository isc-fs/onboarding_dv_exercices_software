"""
ROS 2 bridge for the Mission Control FastAPI backend.

Post-action-decomposition the backend is the sim "operator panel" — the
AMI board + RES buttons stand-in. It does NOT talk to mission_control
directly anymore; it drives the sim uDV emulator (sim_supervisor) over
the sim panel topics, exactly the way the physical AMI/RES drive the real
uDV, and watches the pipeline's /dv/status handshake. mission_control
only ever sees the stock uDV surface.

Hosts a daemon-thread rclpy executor with a single Node that owns:

  * /sim/mission, /sim/intent, /sim/estop publishers (the panel)
  * /dv/status subscriber (the prepare/run handshake the panel waits on)
  * control_node/get_state client (pipeline-active probe)

Sync API for FastAPI handlers (public method names unchanged):

    bridge = RosBridge.get()
    prep = bridge.set_mission("autocross")   # arm + wait for DV_READY
    if prep.success:
        bridge.start_runtime()               # GO + wait for DV_RUNNING
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

from mission_control.interface_contract import (
    DV_FAILED,
    DV_READY,
    DV_RUNNING,
    SIM_INTENT_GO,
    SIM_INTENT_OFF,
    SIM_INTENT_READY,
    TOPIC_DV_STATUS,
    TOPIC_SIM_ESTOP,
    TOPIC_SIM_INTENT,
    TOPIC_SIM_MISSION,
    mission_id_to_ami_index,
)

logger = logging.getLogger(__name__)

# How long the handshake waits for the FIRST /dv/status sample received after
# an intent was published before declaring mission_control unresponsive.
# mission_control heartbeats /dv/status at 10 Hz whenever it is alive, so 3 s
# is ~30 missed beats — well past DDS discovery jitter, well short of the
# prepare timeout. Without this, a frozen pipeline (e.g. no /clock under
# use_sim_time) is indistinguishable from a slow prepare for 270 s.
DV_HEARTBEAT_TIMEOUT_S: float = 3.0

# Mirrors pipeline/mode_manager/mode_manager/mode_registry.py
_MISSION_NAME_TO_ID: dict[str, int] = {
    "trackdrive": 1,
    "autocross": 2,
    "accel": 3,
    "skidpad": 4,
    "scruti": 5,
}


@dataclass
class SetMissionOutcome:
    """Result of bridge.set_mission() (prepare phase)."""

    success: bool
    message: str

    @property
    def ready(self) -> bool:
        """Alias for older call sites that checked `.ready`."""
        return self.success


@dataclass
class RuntimeControlOutcome:
    """Result of bridge.start_runtime() (go phase)."""

    success: bool
    message: str


# Backward-compatible alias
StartMissionOutcome = SetMissionOutcome


class RosBridge:
    """Singleton rclpy host for the FastAPI backend (the sim panel)."""

    _instance: Optional["RosBridge"] = None
    _lock = threading.Lock()

    @classmethod
    def start(cls) -> "RosBridge":
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
                cls._instance._spin_up()
            return cls._instance

    @classmethod
    def get(cls) -> "RosBridge":
        with cls._lock:
            if cls._instance is None:
                raise RuntimeError(
                    "RosBridge.start() has not been called yet. "
                    "FastAPI startup handler must call it once."
                )
            return cls._instance

    @classmethod
    def shutdown(cls) -> None:
        with cls._lock:
            if cls._instance is None:
                return
            cls._instance._spin_down()
            cls._instance = None

    def __init__(self) -> None:
        self._rclpy = None
        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._mission_pub = None
        self._intent_pub = None
        self._estop_pub = None
        self._control_get_state_client = None
        self._ready_event = threading.Event()
        self._control_state_cache: tuple[Optional[int], float] = (None, 0.0)
        self._control_state_cache_ttl_s: float = 0.5
        # Latest /dv/status byte (set on the executor thread; int reads are
        # atomic under the GIL so no lock needed).
        self._dv_status: Optional[int] = None
        # Number of /dv/status samples received so far. The handshake waits
        # snapshot this before publishing an intent and only accept samples
        # that arrived afterwards. /dv/status is TRANSIENT_LOCAL, so the byte
        # left over from the previous session (typically DV_RUNNING) is
        # otherwise "already satisfied" the instant READY/GO go out — which
        # is how a completely frozen pipeline used to report "Session
        # started" without a single node having reacted.
        self._dv_status_seq: int = 0
        self._bridge_lock = threading.Lock()

    def _spin_up(self) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from std_msgs.msg import Bool, Int32, UInt8
        from lifecycle_msgs.srv import GetState

        self._rclpy = rclpy
        self._Bool = Bool
        self._Int32 = Int32
        self._UInt8 = UInt8
        self._GetState = GetState

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        rclpy.init()
        self._node = Node("mission_control_backend_ros_bridge")
        self._mission_pub = self._node.create_publisher(
            Int32, TOPIC_SIM_MISSION, latched)
        self._intent_pub = self._node.create_publisher(
            UInt8, TOPIC_SIM_INTENT, latched)
        self._estop_pub = self._node.create_publisher(
            Bool, TOPIC_SIM_ESTOP, latched)
        self._node.create_subscription(
            UInt8, TOPIC_DV_STATUS, self._on_dv_status, latched)
        self._control_get_state_client = self._node.create_client(
            GetState, "/control_node/get_state")

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)

        def _spin_loop() -> None:
            try:
                self._ready_event.set()
                self._executor.spin()
            except Exception:
                logger.exception("RosBridge spin loop crashed")
            finally:
                logger.info("RosBridge spin loop exited")

        self._spin_thread = threading.Thread(
            target=_spin_loop, name="RosBridge-spin", daemon=True,
        )
        self._spin_thread.start()
        self._ready_event.wait(timeout=5.0)
        logger.info("RosBridge ready (Node spinning)")

    def _spin_down(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()
        if self._rclpy is not None:
            try:
                self._rclpy.try_shutdown()
            except Exception:
                pass
        self._executor = None
        self._spin_thread = None
        self._node = None
        self._mission_pub = None
        self._intent_pub = None
        self._estop_pub = None
        self._control_get_state_client = None
        self._control_state_cache = (None, 0.0)
        self._dv_status = None
        self._dv_status_seq = 0
        self._rclpy = None

    # ------------------------------------------------------------------
    def _on_dv_status(self, msg) -> None:
        # Status first, then the sequence bump: a reader that observes the
        # new seq is guaranteed to read the matching (or a newer) status.
        self._dv_status = int(msg.data)
        self._dv_status_seq += 1

    def _publish_intent(self, intent: int) -> None:
        if self._intent_pub is not None:
            self._intent_pub.publish(self._UInt8(data=int(intent)))

    def _wait_for_dv_status(
        self, targets: set[int], timeout_s: float,
        fail_on: Optional[set[int]] = None,
        after_seq: Optional[int] = None,
    ) -> tuple[bool, str]:
        """Wait for /dv/status (cached on the spin thread) to reach `targets`.

        `after_seq` — pass the `_dv_status_seq` snapshotted *before* the
        intent was published. Only samples received after it count, so the
        latched byte from a previous session can never satisfy the wait. If
        no sample at all arrives within DV_HEARTBEAT_TIMEOUT_S the pipeline
        is treated as unresponsive and the wait fails early.

        Returns (ok, reason); `reason` is human-readable on failure.
        """
        start = time.monotonic()
        deadline = start + timeout_s
        heartbeat_deadline = start + min(DV_HEARTBEAT_TIMEOUT_S, timeout_s)
        while time.monotonic() < deadline:
            seq = self._dv_status_seq
            status = self._dv_status
            fresh = after_seq is None or seq > after_seq
            if not fresh:
                if time.monotonic() >= heartbeat_deadline:
                    return False, (
                        "no /dv/status heartbeat from mission_control within "
                        f"{DV_HEARTBEAT_TIMEOUT_S:.0f}s — pipeline unresponsive "
                        "(is /clock being published? are the stack nodes alive?)"
                    )
            elif status in targets:
                return True, "ok"
            elif fail_on and status in fail_on:
                return False, f"/dv/status reported {status} (failed)"
            time.sleep(0.05)
        return False, (
            f"timed out after {timeout_s:.0f}s (last /dv/status={self._dv_status})"
        )

    def is_action_server_available(self, timeout_s: float = 0.0) -> bool:
        """Back-compat probe — now "is the sim panel bridge up?"."""
        return self._node is not None

    def is_pipeline_active(self) -> bool:
        """Return True iff control_node is in lifecycle state `active`."""
        if self._control_get_state_client is None:
            return False
        now = time.monotonic()
        cached_state, cached_ts = self._control_state_cache
        if cached_state is not None and \
                (now - cached_ts) < self._control_state_cache_ttl_s:
            return cached_state == 3
        state_id = self._get_control_state_blocking()
        if state_id is not None:
            self._control_state_cache = (state_id, now)
            return state_id == 3
        return cached_state == 3 if cached_state is not None else False

    def _get_control_state_blocking(self) -> Optional[int]:
        if self._control_get_state_client is None:
            return None
        if not self._control_get_state_client.service_is_ready():
            return None
        future = self._control_get_state_client.call_async(
            self._GetState.Request())
        deadline = time.monotonic() + 1.0
        while not future.done():
            if time.monotonic() >= deadline:
                self._control_get_state_client.remove_pending_request(future)
                return None
            time.sleep(0.02)
        result = future.result()
        if result is None:
            return None
        return int(result.current_state.id)

    def set_mission(
        self, mission: str, timeout_s: float = 270.0,
    ) -> SetMissionOutcome:
        """Phase 1 — select the mission + arm (READY), wait for DV_READY."""
        if self._mission_pub is None:
            return SetMissionOutcome(
                success=False,
                message="ros_bridge not started; FastAPI startup did not run",
            )

        if mission == "":
            # Tear down — disarm.
            self._publish_intent(SIM_INTENT_OFF)
            return SetMissionOutcome(success=True, message="torn down")

        mission_id = _MISSION_NAME_TO_ID.get(mission)
        if mission_id is None:
            return SetMissionOutcome(
                success=False,
                message=(
                    f"unknown pipeline mission {mission!r}; expected one of "
                    f"{sorted(_MISSION_NAME_TO_ID.keys())}"
                ),
            )

        ami = mission_id_to_ami_index(mission_id)
        seq0 = self._dv_status_seq
        self._mission_pub.publish(self._Int32(data=int(ami)))
        self._publish_intent(SIM_INTENT_READY)

        ok, why = self._wait_for_dv_status(
            {DV_READY, DV_RUNNING}, timeout_s, fail_on={DV_FAILED},
            after_seq=seq0)
        if ok:
            return SetMissionOutcome(
                success=True, message=f"{mission} prepared (DV_READY)")
        return SetMissionOutcome(
            success=False,
            message=f"{mission} did not reach DV_READY: {why}",
        )

    def start_runtime(self, timeout_s: float = 60.0) -> RuntimeControlOutcome:
        """Phase 2 — GO. Drives the emulator's RES go; waits for DV_RUNNING.

        Control commands then flow from mission_control on /ctrl/cmd; the
        emulator relays them to /fsds/control_command for the UE5 bridge.
        """
        if self._intent_pub is None:
            return RuntimeControlOutcome(
                success=False, message="ros_bridge not started")
        seq0 = self._dv_status_seq
        self._publish_intent(SIM_INTENT_GO)
        ok, why = self._wait_for_dv_status(
            {DV_RUNNING}, timeout_s, fail_on={DV_FAILED}, after_seq=seq0)
        if ok:
            return RuntimeControlOutcome(
                success=True, message="mission running (DV_RUNNING)")
        return RuntimeControlOutcome(
            success=False,
            message=f"did not reach DV_RUNNING: {why}",
        )

    def cancel_runtime(self, timeout_s: float = 5.0) -> None:
        """Drop out of the run (disarm). The reconciler tears autonomy down."""
        with self._bridge_lock:
            self._publish_intent(SIM_INTENT_OFF)

    def stop_mission(self, timeout_s: float = 270.0) -> SetMissionOutcome:
        """Tear down: disarm the panel (intent OFF)."""
        self.cancel_runtime()
        return self.set_mission("", timeout_s=timeout_s)

    def run_mission(
        self, mission: str,
        prepare_timeout_s: float = 270.0,
        runtime_timeout_s: float = 60.0,
    ) -> tuple[SetMissionOutcome, Optional[RuntimeControlOutcome]]:
        """Prepare then immediately go (no EBS handling)."""
        prep = self.set_mission(mission, timeout_s=prepare_timeout_s)
        if not prep.success:
            return prep, None
        runtime = self.start_runtime(timeout_s=runtime_timeout_s)
        return prep, runtime

    def start_mission(
        self, mission: str, timeout_s: float = 270.0,
    ) -> SetMissionOutcome:
        """Prepare only, or tear down when mission is ''."""
        if mission == "":
            return self.stop_mission(timeout_s=timeout_s)
        return self.set_mission(mission, timeout_s=timeout_s)
