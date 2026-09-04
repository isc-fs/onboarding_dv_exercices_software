"""
CLI for manual mission control without the web backend.

Drives the sim uDV emulator (sim_supervisor_node) exactly the way the
real AMI board + RES buttons drive the uDV: it publishes the sim operator
panel topics and watches the pipeline's /dv/status handshake. It does NOT
talk to mission_control directly — mission_control only ever sees the
stock uDV surface.

Usage:
    ros2 run sim_supervisor supervisor_cli set_mission <mission_id>
    ros2 run sim_supervisor supervisor_cli start_mission
    ros2 run sim_supervisor supervisor_cli run <mission_id>
    ros2 run sim_supervisor supervisor_cli stop
    ros2 run sim_supervisor supervisor_cli estop

Mission IDs: 1=trackdrive, 2=autocross, 3=accel, 4=skidpad, 5=scruti,
0=tear down.

  set_mission  → /sim/mission + /sim/intent=READY; waits for DV_READY
                 (mission_control configures the autonomy).
  start_mission→ /sim/intent=GO; waits for DV_RUNNING (activate + run).
  run          → set_mission then start_mission.
  stop         → /sim/intent=OFF (tear down).
  estop        → /sim/estop=true (emergency).
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Bool, Int32, UInt8

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


# Latched so the emulator gets the last command even if it (re)joins, and
# so a one-shot publish survives long enough to be delivered.
_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_DV_STATUS_NAME = {
    0: "IDLE", 1: "PREPARING", 2: "READY", 3: "RUNNING",
    4: "FINISHED", 5: "EMERGENCY", 6: "FAILED",
}


class SupervisorCLI:
    """Terminal sim operator panel — publishes /sim/*, watches /dv/status."""

    def __init__(self) -> None:
        rclpy.init()
        self._node = rclpy.create_node("supervisor_cli")
        self._log = self._node.get_logger()

        self._mission_pub = self._node.create_publisher(
            Int32, TOPIC_SIM_MISSION, _LATCHED)
        self._intent_pub = self._node.create_publisher(
            UInt8, TOPIC_SIM_INTENT, _LATCHED)
        self._estop_pub = self._node.create_publisher(
            Bool, TOPIC_SIM_ESTOP, _LATCHED)

        self._dv_status: int | None = None
        self._node.create_subscription(
            UInt8, TOPIC_DV_STATUS, self._on_dv_status, _LATCHED)

    def _on_dv_status(self, msg: UInt8) -> None:
        self._dv_status = int(msg.data)

    def _spin(self, seconds: float) -> None:
        """Spin the node for a fixed wall-clock window (delivery + cb)."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def _wait_for_status(self, targets: set[int], timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        last_logged: int | None = None
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._dv_status is not None and self._dv_status != last_logged:
                self._log.info(
                    f"  /dv/status = {_DV_STATUS_NAME.get(self._dv_status, '?')}")
                last_logged = self._dv_status
            if self._dv_status in targets:
                return True
            if DV_FAILED in (self._dv_status,) and DV_FAILED not in targets:
                return False
        return False

    def set_mission(self, mission_id: int, timeout_s: float = 300.0) -> bool:
        ami = mission_id_to_ami_index(mission_id) if mission_id else 0
        self._log.info(
            f"set_mission: mission_id={mission_id} → AMI index {ami}; arming")
        self._mission_pub.publish(Int32(data=int(ami)))
        self._intent_pub.publish(UInt8(data=int(SIM_INTENT_READY)))
        if mission_id == 0:
            self._intent_pub.publish(UInt8(data=int(SIM_INTENT_OFF)))
            self._spin(0.5)
            self._log.info("OK: torn down")
            return True
        if self._wait_for_status({DV_READY, DV_RUNNING}, timeout_s):
            self._log.info("OK: autonomy prepared (DV_READY)")
            return True
        self._log.error("Failed: pipeline did not reach DV_READY")
        return False

    def start_mission(self, timeout_s: float = 60.0) -> bool:
        self._log.info("start_mission: GO")
        self._intent_pub.publish(UInt8(data=int(SIM_INTENT_GO)))
        if self._wait_for_status({DV_RUNNING}, timeout_s):
            self._log.info("OK: mission running (DV_RUNNING)")
            return True
        self._log.error("Failed: pipeline did not reach DV_RUNNING")
        return False

    def run_mission(self, mission_id: int) -> bool:
        if not self.set_mission(mission_id):
            return False
        time.sleep(0.5)
        return self.start_mission()

    def stop(self) -> bool:
        self._log.info("stop: disarming (intent OFF)")
        self._intent_pub.publish(UInt8(data=int(SIM_INTENT_OFF)))
        self._spin(0.5)
        return True

    def estop(self) -> bool:
        self._log.warning("estop: asserting /sim/estop")
        self._estop_pub.publish(Bool(data=True))
        self._spin(0.5)
        return True

    def cleanup(self) -> None:
        self._node.destroy_node()
        rclpy.shutdown()


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
        elif cmd in ("start_mission", "start", "runtime"):
            ok = cli.start_mission()
        elif cmd == "run":
            if len(sys.argv) < 3:
                print("Error: mission_id required")
                sys.exit(1)
            ok = cli.run_mission(int(sys.argv[2]))
        elif cmd == "stop":
            ok = cli.stop()
        elif cmd == "estop":
            ok = cli.estop()
        else:
            print(f"Unknown command: {cmd}")
            print("Valid: set_mission, start_mission, run, stop, estop")
            sys.exit(1)

        sys.exit(0 if ok else 1)
    except Exception as exc:  # noqa: BLE001
        print(f"Error: {exc}")
        sys.exit(1)
    finally:
        if cli is not None:
            cli.cleanup()


if __name__ == "__main__":
    main()
