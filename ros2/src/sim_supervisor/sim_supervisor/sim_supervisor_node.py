"""
sim_supervisor_node — the sim uDV emulator.

On the real car the uDV is a micro-ROS endpoint (USB CDC). It owns:

  * the AS state machine (+ the AMI mission select / RES go+e-stop inputs)
  * the EBS plumbing
  * relaying control commands to the powertrain
  * publishing its sensors (/imu, steering, wheel speed)

In sim there is no microcontroller, no buttons, no plumbing. This node
fakes all of it so mission_control sees an IDENTICAL stock-typed surface
in sim and on the car — the same reconciler runs against both. **It is
sim-only** and is not launched on the real-car compose stack.

It speaks the stock interface in mission_control.interface_contract:

  emulator → mission_control (uplink):
    /assi/state  (UInt8)  AS state machine — from as_state_machine.py
    /ami/mission (Int32)  selected AMI mission index
  emulator ← mission_control (downlink):
    /dv/status   (UInt8)  pipeline lifecycle handshake (gates "go")
    /ctrl/cmd    (Twist)  normalised command → relayed to the bridge
    /force_ebs   (SetBool srv, served here) → latched /signal/ebs

The sim "operator panel" (the AMI board + RES buttons) is driven by the
backend / CLI over sim-only topics (Linux↔Linux, not the stock uDV
surface): /sim/mission (Int32), /sim/intent (UInt8: OFF/READY/GO),
/sim/estop (Bool).

It also keeps emulating the uDV's /odom publication (the Python
OdometryFilter, dormant by default when odometry_filter_node owns /odom).
"""

from __future__ import annotations

import time

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty as EmptyMsg, Float32, Int32, UInt8
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from fs_msgs.msg import ControlCommand

from mission_control.interface_contract import (
    AS_DRIVING,
    DV_IDLE,
    SERVICE_FORCE_EBS,
    TOPIC_AMI_MISSION,
    TOPIC_ASSI_STATE,
    TOPIC_CTRL_CMD,
    TOPIC_DV_STATUS,
    TOPIC_SIM_ESTOP,
    TOPIC_SIM_INTENT,
    TOPIC_SIM_MISSION,
    ami_index_to_mission_id,
)
from mission_control.interface_qos import UPLINK_QOS
from sim_supervisor.as_state_machine import OperatorIntent, next_as_state
from sim_supervisor.odometry import OdometryFilter


# /odom publication rate. 100 Hz target — gives the 40 Hz controller
# fresh data every tick with margin, doesn't burn the CPU. Decoupled
# from the IMU subscription rate (which is the BMI088's native ~400 Hz).
ODOM_PUBLISH_HZ: float = 100.0

# Take every Nth IMU sample, discard the rest before pushing into the
# OdometryFilter. Default 1 (= no decimation, integrate every sample
# at the BMI088's native ~400 Hz rate) is the production setting —
# matches what the uDV firmware does on the real IFS-08 and preserves
# the sim/real algorithm equivalence the module docstring promises.
#
# Earlier (briefly, in #426) we set this to 4 to drop sim_supervisor
# CPU from 93 % to ~25 %, reasoning that the controller only ticks at
# 40 Hz so 100 Hz integration was enough. That broke parity: the real
# car's uDV consumes IMU at 400 Hz, so a sim filter integrating at
# 100 Hz no longer mirrors the same dynamic. Revert.
#
# If sim_supervisor CPU becomes a real problem again, the right fix
# is to port OdometryFilter to C++ (rclpy callback overhead at 400 Hz
# is the dominant cost, not the integration math). #385 still tracks
# the "quantify whether 100 Hz loses meaningful filter quality" A/B
# question — if the answer is "no", we can re-enable decimation AND
# match it on the real car for symmetric reduced rate. Until then,
# stay at 1.
IMU_DECIMATION: int = 1

# AS-state machine tick + /assi/state heartbeat cadence. >= the 2 Hz the
# interface contract requires for the heartbeat; mission_control treats a
# stale /assi/state as the uDV being dead.
AS_PUBLISH_HZ: float = 10.0

# /dv/status liveness: a stale downlink means mission_control / the link
# is dead, so the emulator treats the status as DV_IDLE (go is gated off,
# nothing drives). Must comfortably exceed mission_control's publish
# period (>= 2 Hz → 0.5 s).
DV_STATUS_STALE_S: float = 1.5


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# /ctrl/cmd is a best-effort 40 Hz stream — match mission_control's pub.
CMD_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class SimSupervisorNode(LifecycleNode):
    """Sim-only uDV emulator. See module docstring."""

    NODE_NAME = "sim_supervisor_node"

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)

        # --- emulator I/O (configured in on_configure) ---
        self._ctrl_cmd_sub = None
        self._dv_status_sub = None
        self._sim_mission_sub = None
        self._sim_intent_sub = None
        self._sim_estop_sub = None
        self._assi_state_pub = None
        self._ami_mission_pub = None
        self._force_ebs_srv = None
        self._as_tick_timer = None

        self._control_pub = None
        self._ebs_pub = None
        self._ebs_reset_pub = None

        # --- emulator state ---
        # operator panel inputs (from the backend/CLI over /sim/*).
        self._operator_intent: int = int(OperatorIntent.OFF)
        self._mission_index: int = 0          # raw AMI index on /sim/mission
        self._estop: bool = False
        # AS state machine output + the latest pipeline handshake.
        self._as_state: int = 0               # AS_OFF
        self._dv_status: int = DV_IDLE
        self._dv_status_stamp: float = 0.0

        # IMU sample counter, modulo IMU_DECIMATION.
        self._imu_sample_idx: int = 0

        self._cb_group = ReentrantCallbackGroup()

        # Odometry filter (Phase 1 of the /odom split — see
        # docs/AUTONOMY.md §"Open questions" Q1). Subscribes to /imu and
        # /motor_rpm, publishes /odom at ODOM_PUBLISH_HZ. The emulator is
        # the natural owner because on the real car the uDV publishes
        # /odom from the same input set.
        self._odom_filter: OdometryFilter | None = None
        self._odom_pub = None
        self._odom_pub_timer = None
        self._sub_steering = None
        self._sub_brake = None
        self._yaw_residual_pub = None
        self._slip_flag_pub = None
        self._effective_alpha_pub = None
        self._odom_tf_broadcaster: TransformBroadcaster | None = None
        self._sub_imu = None
        self._sub_rpm = None
        self._odom_first_publish_logged: bool = False

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure: creating emulator I/O")

        # External filter opt-out (#431/Phase 2). When True, this node
        # skips its own /odom publishing path entirely — the C++
        # odometry_filter_node owns those surfaces instead. Default True.
        self.declare_parameter("use_external_odometry_filter", True)
        self._use_external_filter = (
            self.get_parameter("use_external_odometry_filter").value
        )
        if self._use_external_filter:
            self.get_logger().info(
                "use_external_odometry_filter=True — /odom + /odom_diag/* "
                "+ odom→base_link TF owned by odometry_filter_node (C++)")

        # --- uDV uplink publishers (the stock interface) ---
        # BEST_EFFORT/VOLATILE (UPLINK_QOS), matching the real uDV's
        # micro-ROS heartbeat idiom so mission_control's reader connects
        # identically in sim and on the car. Published every _as_tick, so
        # the steady heartbeat — not durability — covers late join. See
        # mission_control.interface_qos.
        self._assi_state_pub = self.create_lifecycle_publisher(
            UInt8, TOPIC_ASSI_STATE, UPLINK_QOS)
        self._ami_mission_pub = self.create_lifecycle_publisher(
            Int32, TOPIC_AMI_MISSION, UPLINK_QOS)

        # --- downlink subscriptions ---
        self._ctrl_cmd_sub = self.create_subscription(
            Twist, TOPIC_CTRL_CMD, self._on_ctrl_cmd, CMD_QOS,
            callback_group=self._cb_group)
        self._dv_status_sub = self.create_subscription(
            UInt8, TOPIC_DV_STATUS, self._on_dv_status, LATCHED_QOS,
            callback_group=self._cb_group)
        self._force_ebs_srv = self.create_service(
            SetBool, SERVICE_FORCE_EBS, self._on_force_ebs,
            callback_group=self._cb_group)

        # --- sim operator panel (backend/CLI → emulator) ---
        self._sim_mission_sub = self.create_subscription(
            Int32, TOPIC_SIM_MISSION, self._on_sim_mission, LATCHED_QOS,
            callback_group=self._cb_group)
        self._sim_intent_sub = self.create_subscription(
            UInt8, TOPIC_SIM_INTENT, self._on_sim_intent, LATCHED_QOS,
            callback_group=self._cb_group)
        self._sim_estop_sub = self.create_subscription(
            Bool, TOPIC_SIM_ESTOP, self._on_sim_estop, LATCHED_QOS,
            callback_group=self._cb_group)

        # --- bridge outputs ---
        self._control_pub = self.create_lifecycle_publisher(
            ControlCommand, "/fsds/control_command", 10)
        self._ebs_pub = self.create_lifecycle_publisher(
            EmptyMsg, "/signal/ebs", LATCHED_QOS)
        self._ebs_reset_pub = self.create_lifecycle_publisher(
            EmptyMsg, "/signal/ebs_reset", LATCHED_QOS)

        # /odom infrastructure — skipped when the C++ filter owns it.
        if not self._use_external_filter:
            self._odom_filter = OdometryFilter()
            self._odom_pub = self.create_lifecycle_publisher(
                Odometry, "/odom", 50)
            self._odom_tf_broadcaster = TransformBroadcaster(self)
            self._yaw_residual_pub = self.create_lifecycle_publisher(
                Float32, "/odom_diag/yaw_residual_rad_s", 10)
            self._slip_flag_pub = self.create_lifecycle_publisher(
                Bool, "/odom_diag/slip_flag", 10)
            self._effective_alpha_pub = self.create_lifecycle_publisher(
                Float32, "/odom_diag/effective_alpha_vx", 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"on_activate: starting AS-state tick ({AS_PUBLISH_HZ:.0f} Hz)")

        # AS-state machine heartbeat — publishes /assi/state + /ami/mission.
        self._as_tick_timer = self.create_timer(
            1.0 / AS_PUBLISH_HZ, self._as_tick, callback_group=self._cb_group)

        # Reset the filter so a deactivate→activate cycle starts a fresh
        # stationary calibration.
        if self._odom_filter is not None:
            self._odom_filter.reset()
        self._odom_first_publish_logged = False

        if not self._use_external_filter:
            imu_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=2000,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._sub_imu = self.create_subscription(
                Imu, "/imu", self._on_imu, imu_qos,
                callback_group=self._cb_group)
            rpm_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._sub_rpm = self.create_subscription(
                Float32, "/motor_rpm", self._on_rpm, rpm_qos,
                callback_group=self._cb_group)
            self._sub_steering = self.create_subscription(
                Float32, "/steering_angle", self._on_steering, rpm_qos,
                callback_group=self._cb_group)
            self._sub_brake = self.create_subscription(
                Float32, "/brake_pressure", self._on_brake, rpm_qos,
                callback_group=self._cb_group)
            self._odom_pub_timer = self.create_timer(
                1.0 / ODOM_PUBLISH_HZ, self._publish_odom,
                callback_group=self._cb_group)

        # LifecyclePublisher.publish() is a silent no-op until the base
        # class on_activate() enables the node's managed publishers, so
        # anything that must go out exactly once on activation has to be
        # published AFTER super().on_activate().
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret

        # Latched /signal/ebs_reset (post-#384). On the real car the uDV
        # firmware clears the EBS gate at power-up; the emulator does the
        # same the moment its lifecycle goes active. Without this, the
        # bridge's ebs_triggered_ gate stays latched from a previous run
        # and every relayed control command is silently dropped.
        if self._ebs_reset_pub is not None:
            self._ebs_reset_pub.publish(EmptyMsg())

        return ret

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate: stopping AS tick + /odom subs")
        if self._as_tick_timer is not None:
            self.destroy_timer(self._as_tick_timer)
            self._as_tick_timer = None
        if self._odom_pub_timer is not None:
            self.destroy_timer(self._odom_pub_timer)
            self._odom_pub_timer = None
        for sub in (self._sub_imu, self._sub_rpm,
                    self._sub_steering, self._sub_brake):
            if sub is not None:
                self.destroy_subscription(sub)
        self._sub_imu = None
        self._sub_rpm = None
        self._sub_steering = None
        self._sub_brake = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup: tearing down I/O")
        if self._as_tick_timer is not None:
            self.destroy_timer(self._as_tick_timer)
            self._as_tick_timer = None
        for sub in (self._ctrl_cmd_sub, self._dv_status_sub,
                    self._sim_mission_sub, self._sim_intent_sub,
                    self._sim_estop_sub):
            if sub is not None:
                self.destroy_subscription(sub)
        self._ctrl_cmd_sub = self._dv_status_sub = None
        self._sim_mission_sub = self._sim_intent_sub = self._sim_estop_sub = None
        self._force_ebs_srv = None
        self._assi_state_pub = None
        self._ami_mission_pub = None
        self._control_pub = None
        self._ebs_pub = None
        self._ebs_reset_pub = None
        if self._odom_pub_timer is not None:
            self.destroy_timer(self._odom_pub_timer)
            self._odom_pub_timer = None
        self._odom_pub = None
        self._odom_tf_broadcaster = None
        self._yaw_residual_pub = None
        self._slip_flag_pub = None
        self._effective_alpha_pub = None
        self._odom_filter = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown")
        return TransitionCallbackReturn.SUCCESS

    # ==================================================================
    # AS state machine + emulator surface
    # ==================================================================
    def _on_dv_status(self, msg: UInt8) -> None:
        self._dv_status = int(msg.data)
        self._dv_status_stamp = time.monotonic()

    def _on_sim_mission(self, msg: Int32) -> None:
        if int(msg.data) != self._mission_index:
            self.get_logger().info(f"/sim/mission → {msg.data}")
        self._mission_index = int(msg.data)

    def _on_sim_intent(self, msg: UInt8) -> None:
        if int(msg.data) != self._operator_intent:
            self.get_logger().info(f"/sim/intent → {msg.data}")
        self._operator_intent = int(msg.data)

    def _on_sim_estop(self, msg: Bool) -> None:
        self._estop = bool(msg.data)
        if self._estop:
            self.get_logger().warn("/sim/estop asserted")

    def _effective_dv_status(self) -> int:
        """Latest /dv/status, or DV_IDLE if the downlink is stale."""
        if (time.monotonic() - self._dv_status_stamp) > DV_STATUS_STALE_S:
            return DV_IDLE
        return self._dv_status

    def _as_tick(self) -> None:
        """Advance the AS state machine + publish /assi/state + /ami/mission."""
        mission_runnable = ami_index_to_mission_id(self._mission_index) > 0
        new_state = next_as_state(
            self._as_state,
            self._operator_intent,
            self._estop,
            self._effective_dv_status(),
            mission_runnable,
        )
        if new_state != self._as_state:
            self.get_logger().info(
                f"AS state {self._as_state} → {new_state}")
            self._as_state = new_state

        if self._assi_state_pub is not None:
            self._assi_state_pub.publish(UInt8(data=int(self._as_state)))
        if self._ami_mission_pub is not None:
            self._ami_mission_pub.publish(Int32(data=int(self._mission_index)))

    def _on_ctrl_cmd(self, msg: Twist) -> None:
        """Relay the normalised /ctrl/cmd to the bridge — only while Driving.

        Mirrors the uDV gating its powertrain on AS Driving: the command
        is applied only in AS_DRIVING. linear.x is the signed drive demand
        (throttle − brake/regen, [-1, 1]) and is split back into the two
        non-negative ControlCommand channels; steering = angular.z
        ([-1, 1]). The sim bridge consumes the normalised ControlCommand
        directly (no scaling — that's the uDV's job on the real car).
        """
        if self._as_state != AS_DRIVING or self._control_pub is None:
            return
        x = float(msg.linear.x)
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.throttle = max(x, 0.0)
        cmd.brake = max(-x, 0.0)
        cmd.steering = float(msg.angular.z)
        self._control_pub.publish(cmd)

    def _on_force_ebs(self, request, response):
        """uDV /force_ebs server — mission_control requests EBS here."""
        if request.data and self._ebs_pub is not None:
            self.get_logger().warn(
                "/force_ebs requested — publishing latched /signal/ebs")
            self._ebs_pub.publish(EmptyMsg())
        response.success = True
        response.message = "EBS latched" if request.data else "no-op"
        return response

    # ==================================================================
    # /odom — IMU + RPM → dead-reckoning Odometry
    # ==================================================================
    def _on_imu(self, msg: Imu) -> None:
        """Drive the filter's predict step.

        Decimates the 400 Hz IMU stream by IMU_DECIMATION before pushing
        into the filter — see the module-level constant for the CPU
        rationale (#385). Bridge subscription stays at full depth so the
        unused samples flow through DDS at zero cost to us; we just skip
        the np.array construction + push_imu call for the dropped 3 of 4.
        """
        if self._odom_filter is None:
            return
        self._imu_sample_idx += 1
        if self._imu_sample_idx % IMU_DECIMATION:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        self._odom_filter.push_imu(t, accel, gyro)

    def _on_rpm(self, msg: Float32) -> None:
        """Drive the filter's correction step."""
        if self._odom_filter is None:
            return
        # Wall-clock timestamp — the bridge publishes Float32 with no
        # header.stamp on /motor_rpm, so we mark received-time here.
        # Used for staleness inside the filter.
        self._odom_filter.push_rpm(time.monotonic(), float(msg.data))

    def _on_steering(self, msg: Float32) -> None:
        """Cache the latest front-wheel angle (rad). Used inside the
        filter's push_imu step for the kinematic-bicycle yaw cross-
        check (#383)."""
        if self._odom_filter is None:
            return
        self._odom_filter.push_steering(time.monotonic(), float(msg.data))

    def _on_brake(self, msg: Float32) -> None:
        """Cache the latest brake authority [0, 1]. Used inside push_rpm
        to scale α_vx during brake events (#383)."""
        if self._odom_filter is None:
            return
        self._odom_filter.push_brake(time.monotonic(), float(msg.data))

    def _publish_odom(self) -> None:
        """Timer-driven /odom topic + odom→base_link TF emission.
        Skips while the filter is still in stationary calibration
        (first ~3 s after activate)."""
        if (self._odom_filter is None
                or self._odom_pub is None
                or self._odom_tf_broadcaster is None):
            return
        if not self._odom_filter.is_calibrated():
            return

        s = self._odom_filter.state
        now = self.get_clock().now().to_msg()

        if not self._odom_first_publish_logged:
            self.get_logger().info(
                "/odom first publish — IMU+RPM filter calibrated")
            self._odom_first_publish_logged = True

        # 2D yaw → unit quaternion (axis-z) used by both the Odometry
        # message and the TF broadcast.
        half = 0.5 * s.yaw
        qw = float(np.cos(half))
        qz = float(np.sin(half))

        # nav_msgs/Odometry: pose in header.frame_id (odom), twist in
        # child_frame_id (base_link). REP-103 axes.
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = s.x
        msg.pose.pose.position.y = s.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = qw
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.twist.twist.linear.x = s.vx
        msg.twist.twist.linear.y = s.vy
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.z = s.yaw_rate
        self._odom_pub.publish(msg)

        # odom → base_link TF (Phase 2 — #382). Same pose, broadcast at
        # the 100 Hz publish rate so downstream TF lookups see a high-rate
        # dead-reckoning leaf.
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = s.x
        tf.transform.translation.y = s.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = qw
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = qz
        self._odom_tf_broadcaster.sendTransform(tf)

        # Phase 3 (#383) diagnostics.
        diag = self._odom_filter.diagnostics
        if self._yaw_residual_pub is not None:
            self._yaw_residual_pub.publish(
                Float32(data=float(diag.yaw_residual_rad_s)))
        if self._slip_flag_pub is not None:
            self._slip_flag_pub.publish(Bool(data=bool(diag.slip_flag)))
        if self._effective_alpha_pub is not None:
            self._effective_alpha_pub.publish(
                Float32(data=float(diag.effective_alpha_vx)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimSupervisorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
