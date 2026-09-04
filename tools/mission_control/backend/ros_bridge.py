"""
ROS 2 bridge for the Mission Control FastAPI backend.

Hosts a daemon-thread rclpy executor with a single Node that owns:

  * SetMission client → mission_control_node (Phase 1: prepare/configure)
  * RuntimeControl client → mission_control_node (Phase 2: activate + run)

Sync API for FastAPI handlers:

    bridge = RosBridge.get()
    prep = bridge.set_mission("autocross")
    if prep.success:
        bridge.start_runtime()  # after EBS release
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)

# Mirrors pipeline/mode_manager/mode_manager/mode_registry.py
_MISSION_NAME_TO_ID: dict[str, int] = {
    "trackdrive": 1,
    "autocross": 2,
    "accel": 3,
    "skidpad": 4,
    "scruti": 5,
}

_MC_SET_MISSION_ACTION = "/mission_control_node/set_mission"
_MC_RUNTIME_CONTROL_ACTION = "/mission_control_node/runtime_control"


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
    """Result of bridge.start_runtime() goal acceptance."""

    success: bool
    message: str


# Backward-compatible alias
StartMissionOutcome = SetMissionOutcome


class RosBridge:
    """Singleton rclpy host for the FastAPI backend."""

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
        self._set_mission_client = None
        self._runtime_control_client = None
        self._runtime_goal_handle = None
        self._control_get_state_client = None
        self._ready_event = threading.Event()
        self._control_state_cache: tuple[Optional[int], float] = (None, 0.0)
        self._control_state_cache_ttl_s: float = 0.5
        self._bridge_lock = threading.Lock()

    def _spin_up(self) -> None:
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from dv_msgs.action import SetMission, RuntimeControl
        from lifecycle_msgs.srv import GetState

        self._rclpy = rclpy
        self._SetMission = SetMission
        self._RuntimeControl = RuntimeControl
        self._GetState = GetState

        rclpy.init()
        self._node = Node("mission_control_backend_ros_bridge")
        self._set_mission_client = ActionClient(
            self._node,
            SetMission,
            _MC_SET_MISSION_ACTION,
        )
        self._runtime_control_client = ActionClient(
            self._node,
            RuntimeControl,
            _MC_RUNTIME_CONTROL_ACTION,
        )
        self._control_get_state_client = self._node.create_client(
            GetState,
            "/control_node/get_state",
        )

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
        with self._bridge_lock:
            self._runtime_goal_handle = None
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
        self._set_mission_client = None
        self._runtime_control_client = None
        self._control_get_state_client = None
        self._control_state_cache = (None, 0.0)
        self._rclpy = None

    def is_action_server_available(self, timeout_s: float = 0.0) -> bool:
        """Probe mission_control_node's set_mission action server."""
        if self._set_mission_client is None:
            return False
        return self._set_mission_client.wait_for_server(timeout_sec=timeout_s)

    def is_pipeline_active(self) -> bool:
        """Return True iff control_node is in lifecycle state `active`."""
        if self._control_get_state_client is None:
            return False

        now = time.monotonic()
        cached_state, cached_ts = self._control_state_cache
        if cached_state is not None and (now - cached_ts) < self._control_state_cache_ttl_s:
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
        future = self._control_get_state_client.call_async(self._GetState.Request())
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
        self,
        mission: str,
        timeout_s: float = 270.0,
    ) -> SetMissionOutcome:
        """Phase 1 — prepare autonomy (CONFIGURE only)."""
        if self._set_mission_client is None:
            return SetMissionOutcome(
                success=False,
                message="ros_bridge not started; FastAPI startup did not run",
            )

        if mission == "":
            mission_id = 0
        else:
            mission_id = _MISSION_NAME_TO_ID.get(mission)
            if mission_id is None:
                return SetMissionOutcome(
                    success=False,
                    message=(
                        f"unknown mission {mission!r}; expected one of "
                        f"{sorted(_MISSION_NAME_TO_ID.keys())}"
                    ),
                )

        if not self._set_mission_client.wait_for_server(timeout_sec=5.0):
            return SetMissionOutcome(
                success=False,
                message=(
                    f"{_MC_SET_MISSION_ACTION} unavailable; "
                    "is mission_control_node active?"
                ),
            )

        goal = self._SetMission.Goal()
        goal.mission_id = mission_id

        send_future = self._set_mission_client.send_goal_async(goal)

        deadline = time.monotonic() + timeout_s
        while not send_future.done():
            if time.monotonic() >= deadline:
                return SetMissionOutcome(
                    success=False,
                    message="SetMission goal acceptance timed out",
                )
            time.sleep(0.05)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return SetMissionOutcome(
                success=False,
                message="sim_supervisor rejected the SetMission goal",
            )

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if time.monotonic() >= deadline:
                return SetMissionOutcome(
                    success=False,
                    message=(
                        f"SetMission did not return within "
                        f"{timeout_s:.0f} s"
                    ),
                )
            time.sleep(0.05)

        wrapper = result_future.result()
        if wrapper is None or wrapper.result is None:
            return SetMissionOutcome(
                success=False,
                message="SetMission completed without a result payload",
            )
        result = wrapper.result
        return SetMissionOutcome(
            success=bool(result.success),
            message=str(result.message),
        )

    def start_runtime(
        self,
        timeout_s: float = 60.0,
    ) -> RuntimeControlOutcome:
        """Phase 2 — activate autonomy and open the control loop.

        Sends RuntimeControl to mission_control_node. Returns once the
        goal is accepted (nodes activating); the action stays open until
        the mission ends. Control commands flow via action feedback and
        sim_supervisor relays them to /fsds/control_command.
        """
        if self._runtime_control_client is None:
            return RuntimeControlOutcome(
                success=False,
                message="ros_bridge not started",
            )

        with self._bridge_lock:
            if self._runtime_goal_handle is not None:
                self._cancel_runtime_control_locked()

            if not self._runtime_control_client.wait_for_server(timeout_sec=5.0):
                return RuntimeControlOutcome(
                    success=False,
                    message=(
                        f"{_MC_RUNTIME_CONTROL_ACTION} unavailable; "
                        "is mission_control_node active?"
                    ),
                )

            send_future = self._runtime_control_client.send_goal_async(
                self._RuntimeControl.Goal(),
            )

            deadline = time.monotonic() + timeout_s
            while not send_future.done():
                if time.monotonic() >= deadline:
                    return RuntimeControlOutcome(
                        success=False,
                        message="RuntimeControl goal acceptance timed out",
                    )
                time.sleep(0.05)

            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                return RuntimeControlOutcome(
                    success=False,
                    message="mission_control rejected RuntimeControl goal",
                )

            self._runtime_goal_handle = goal_handle
            goal_handle.get_result_async().add_done_callback(
                self._on_runtime_result_done,
            )

        return RuntimeControlOutcome(
            success=True,
            message="RuntimeControl goal accepted",
        )

    def _on_runtime_result_done(self, result_future) -> None:
        with self._bridge_lock:
            if self._runtime_goal_handle is not None:
                try:
                    wrapper = result_future.result()
                    if wrapper and wrapper.result:
                        logger.info(
                            "RuntimeControl ended: outcome=%s msg=%s",
                            wrapper.result.outcome,
                            wrapper.result.message,
                        )
                except Exception:
                    logger.exception("RuntimeControl result callback failed")
            self._runtime_goal_handle = None

    def cancel_runtime(self, timeout_s: float = 5.0) -> None:
        """Cancel an in-flight RuntimeControl goal (fire-and-forget)."""
        with self._bridge_lock:
            self._cancel_runtime_control_locked(timeout_s)

    def _cancel_runtime_control_locked(self, timeout_s: float = 5.0) -> None:
        gh = self._runtime_goal_handle
        self._runtime_goal_handle = None
        if gh is None:
            return
        try:
            cancel_future = gh.cancel_goal_async()
            deadline = time.monotonic() + timeout_s
            while not cancel_future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
        except Exception:
            logger.exception("RuntimeControl cancel failed")

    def stop_mission(self, timeout_s: float = 270.0) -> SetMissionOutcome:
        """Tear down: cancel RuntimeControl, then SetMission(mission_id=0)."""
        self.cancel_runtime()
        return self.set_mission("", timeout_s=timeout_s)

    def run_mission(
        self,
        mission: str,
        prepare_timeout_s: float = 270.0,
        runtime_timeout_s: float = 60.0,
    ) -> tuple[SetMissionOutcome, Optional[RuntimeControlOutcome]]:
        """Prepare then immediately start runtime (no EBS handling)."""
        prep = self.set_mission(mission, timeout_s=prepare_timeout_s)
        if not prep.success:
            return prep, None
        runtime = self.start_runtime(timeout_s=runtime_timeout_s)
        return prep, runtime

    def start_mission(
        self,
        mission: str,
        timeout_s: float = 270.0,
    ) -> SetMissionOutcome:
        """Prepare only, or tear down when mission is ''."""
        if mission == "":
            return self.stop_mission(timeout_s=timeout_s)
        return self.set_mission(mission, timeout_s=timeout_s)
