"""
CLI for manual mission control without the web backend.

Usage:
    ros2 run sim_supervisor supervisor_cli set_mission <mission_id>
    ros2 run sim_supervisor supervisor_cli start_mission [--wait]
    ros2 run sim_supervisor supervisor_cli run [--wait] <mission_id>

Mission IDs: 1=trackdrive, 2=autocross, 3=accel, 4=skidpad, 5=scruti, 0=tear down.

set_mission  → /mission_control_node/set_mission (prepare / CONFIGURE)
start_mission → /mission_control_node/runtime_control (activate + run)
run          → set_mission then start_mission
"""

from __future__ import annotations

import sys
import time

import rclpy
from dv_msgs.action import RuntimeControl, SetMission
from rclpy.action import ActionClient


_MC_SET_MISSION = "/mission_control_node/set_mission"
_MC_RUNTIME_CONTROL = "/mission_control_node/runtime_control"


class SupervisorCLI:
    """Terminal client matching the web backend's two-phase flow."""

    def __init__(self) -> None:
        rclpy.init()
        self._node = rclpy.create_node("supervisor_cli")
        self._log = self._node.get_logger()

        self._set_client = ActionClient(self._node, SetMission, _MC_SET_MISSION)
        self._runtime_client = ActionClient(
            self._node, RuntimeControl, _MC_RUNTIME_CONTROL,
        )

        if not self._set_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(
                f"{_MC_SET_MISSION} unavailable (is mission_control_node running?)"
            )
        if not self._runtime_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                f"{_MC_RUNTIME_CONTROL} unavailable "
                "(is mission_control_node running?)"
            )

    def set_mission(self, mission_id: int, timeout_s: float = 300.0) -> bool:
        self._log.info(f"SetMission: mission_id={mission_id}")
        goal = SetMission.Goal()
        goal.mission_id = mission_id

        send_fut = self._set_client.send_goal_async(
            goal, feedback_callback=self._on_set_feedback,
        )
        rclpy.spin_until_future_complete(self._node, send_fut, timeout_sec=15.0)
        if not send_fut.done():
            self._log.error("SetMission goal send timed out")
            return False

        gh = send_fut.result()
        if not gh.accepted:
            self._log.error("SetMission goal rejected")
            return False

        result_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_fut, timeout_sec=timeout_s)
        if not result_fut.done():
            self._log.error("SetMission result timed out")
            return False

        result = result_fut.result().result
        if result.success:
            self._log.info(f"OK: {result.message}")
            return True
        self._log.error(f"Failed: {result.message}")
        return False

    def _on_set_feedback(self, fb_msg) -> None:
        self._log.info(f"  [{fb_msg.feedback.stage}]")

    def start_mission(self, *, wait_for_result: bool = False) -> bool:
        self._log.info("RuntimeControl — activating autonomy")
        send_fut = self._runtime_client.send_goal_async(RuntimeControl.Goal())
        rclpy.spin_until_future_complete(self._node, send_fut, timeout_sec=15.0)
        if not send_fut.done():
            self._log.error("RuntimeControl goal send timed out")
            return False

        gh = send_fut.result()
        if not gh.accepted:
            self._log.error("RuntimeControl goal rejected")
            return False

        self._log.info("RuntimeControl accepted — control via feedback relay")

        if not wait_for_result:
            return True

        result_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_fut, timeout_sec=400.0)
        if not result_fut.done():
            self._log.warning("RuntimeControl still running or timed out")
            return False

        result = result_fut.result().result
        self._log.info(
            f"RuntimeControl ended: outcome={result.outcome!r} msg={result.message!r}"
        )
        return True

    def run_mission(
        self, mission_id: int, *, wait_for_result: bool = False,
    ) -> bool:
        if not self.set_mission(mission_id):
            return False
        time.sleep(0.5)
        return self.start_mission(wait_for_result=wait_for_result)

    def cleanup(self) -> None:
        self._node.destroy_node()
        rclpy.shutdown()


def _pop_wait_flag(argv: list[str]) -> tuple[list[str], bool]:
    wait = "--wait" in argv
    rest = [a for a in argv if a != "--wait"]
    return rest, wait


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    cmd = sys.argv[1].lower()
    cli: SupervisorCLI | None = None
    try:
        cli = SupervisorCLI()

        if cmd == "set_mission":
            if len(sys.argv) < 3:
                print("Error: mission_id required")
                sys.exit(1)
            ok = cli.set_mission(int(sys.argv[2]))
            sys.exit(0 if ok else 1)

        if cmd in ("start_mission", "start", "runtime"):
            rest, wait = _pop_wait_flag(sys.argv[2:])
            ok = cli.start_mission(wait_for_result=wait)
            sys.exit(0 if ok else 1)

        if cmd == "run":
            rest, wait = _pop_wait_flag(sys.argv[2:])
            if len(rest) < 1:
                print("Error: mission_id required")
                sys.exit(1)
            ok = cli.run_mission(int(rest[0]), wait_for_result=wait)
            sys.exit(0 if ok else 1)

        print(f"Unknown command: {cmd}")
        print("Valid: set_mission, start_mission, run")
        sys.exit(1)
    except Exception as exc:
        print(f"Error: {exc}")
        sys.exit(1)
    finally:
        if cli is not None:
            cli.cleanup()


if __name__ == "__main__":
    main()
