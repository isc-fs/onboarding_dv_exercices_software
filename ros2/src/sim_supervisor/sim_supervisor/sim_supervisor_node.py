"""
sim_supervisor_node — the DV pipeline's stand-in for the IFS-08 uDV.

On the real car the uDV is a microROS endpoint (USB CDC). It owns:

  * the physical GO button → emits the GO signal into the autonomy
  * the physical RES button + EBS plumbing
  * the DVPC↔uDV action endpoint that the autonomy talks to
  * forwarding control commands to the powertrain

In sim there is no microcontroller, no buttons, no plumbing. This
node fakes all of that so the autonomy stack sees an identical ROS 2
surface in sim and on the real car. **It is sim-only** and is not
launched on the real-car compose stack.

Two-phase action protocol (see docs/AUTONOMY.md §"Runtime action
protocol"). Both action servers live on mission_control_node; this
node is no longer in the action chain — its only autonomy-facing
responsibility is the RuntimeControl feedback relay:

  Phase 1 — SetMission (prepare). Handled entirely by mission_control_node
            (configure / setup of the autonomy lifecycle nodes via
            mode_manager). Backend + CLI send the goal directly.

  Phase 2 — RuntimeControl (activate + run). Also hosted by
            mission_control_node. This node subscribes passively to
            the action's feedback topic and republishes throttle/steer
            onto /fsds/control_command for the UE5 bridge.
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

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty as EmptyMsg, Float32
from tf2_ros import TransformBroadcaster

from fs_msgs.msg import ControlCommand
from dv_msgs.action import RuntimeControl
from dv_msgs.srv import ActivateMode  # noqa: F401 — kept for future use

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


# mission_control_node publishes RuntimeControl feedback here; this
# node subscribes and relays to /fsds/control_command.
_DEFAULT_RUNTIME_CONTROL_FEEDBACK_TOPIC = (
    "/mission_control_node/runtime_control/_action/feedback"
)


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class SimSupervisorNode(LifecycleNode):
    """Sim-only DVPC stand-in. See module docstring."""

    NODE_NAME = "sim_supervisor_node"

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)

        # Configured in on_configure, torn down in on_cleanup.
        self._runtime_feedback_sub = None
        self._control_pub = None
        self._ebs_pub = None
        self._ebs_reset_pub = None

        # Rising-edge latch for /signal/ebs during a RuntimeControl run.
        self._runtime_ebs_latched: bool = False

        # IMU sample counter, modulo IMU_DECIMATION. Pre-decimation
        # the supervisor was integrating ~400 IMU samples/s on a
        # single CPU; counting + dropping in the callback is the
        # cheapest possible throttle.
        self._imu_sample_idx: int = 0

        # Reentrant group for the RuntimeControl feedback relay and
        # the sim-side I/O callbacks. No action server lives on this
        # node anymore (post-refactor mission_control_node hosts both
        # SetMission and RuntimeControl).
        self._cb_group = ReentrantCallbackGroup()

        # Odometry filter (Phase 1 of the /odom split — see
        # docs/AUTONOMY.md §"Open questions" Q1). Subscribes
        # to /imu and /motor_rpm, publishes /odom at ODOM_PUBLISH_HZ.
        # The supervisor is the natural owner because on the real car
        # the uDV (which this node simulates) publishes /odom from the
        # same input set.
        self._odom_filter: OdometryFilter | None = None
        self._odom_pub = None
        self._odom_pub_timer = None
        # Phase 3 (#383) — steering + brake_pressure inputs from the
        # bridge + diagnostic publishers for the OdometryFilter
        # cross-check residuals. Subscriptions are bound to
        # on_activate (they only need to flow when /odom is being
        # published); diagnostic publishers are lifecycle-aware.
        self._sub_steering = None
        self._sub_brake = None
        self._yaw_residual_pub = None
        self._slip_flag_pub = None
        self._effective_alpha_pub = None
        # Phase 2 (#382): supervisor owns the odom→base_link TF
        # broadcast (slam_node stopped doing it in #382 and now
        # publishes map→odom instead, computed from slam_pose ⊖
        # latest /odom). Created in on_configure, used inside
        # _publish_odom.
        self._odom_tf_broadcaster: TransformBroadcaster | None = None
        self._sub_imu = None
        self._sub_rpm = None
        self._odom_first_publish_logged: bool = False

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure: creating I/O")

        # External filter opt-out (#431/Phase 2). When True, this node
        # skips its own /odom publishing path entirely — no filter
        # instance, no IMU/RPM/steering/brake subscriptions, no /odom
        # publisher, no /odom_diag/* publishers, no odom→base_link TF
        # broadcaster. The C++ `odometry_filter_node` owns those
        # surfaces instead. Default True now that the C++ node is
        # the production path; flip to False (or override at launch)
        # to fall back to the Python OdometryFilter if needed.
        self.declare_parameter("use_external_odometry_filter", True)
        self._use_external_filter = (
            self.get_parameter("use_external_odometry_filter").value
        )
        if self._use_external_filter:
            self.get_logger().info(
                "use_external_odometry_filter=True — /odom + /odom_diag/* "
                "+ odom→base_link TF owned by odometry_filter_node (C++)")

        self.declare_parameter(
            "runtime_control_feedback_topic",
            _DEFAULT_RUNTIME_CONTROL_FEEDBACK_TOPIC,
        )
        fb_topic = str(
            self.get_parameter("runtime_control_feedback_topic").value
        )
        self._runtime_feedback_sub = self.create_subscription(
            RuntimeControl.Impl.FeedbackMessage,
            fb_topic,
            self._relay_runtime_control_feedback,
            10,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"RuntimeControl feedback relay: {fb_topic!r} → /fsds/control_command"
        )

        # Output: /fsds/control_command — the bridge subscribes here.
        self._control_pub = self.create_lifecycle_publisher(
            ControlCommand, "/fsds/control_command", 10,
        )

        # Latched EBS + EBS reset onto the bridge.
        self._ebs_pub = self.create_lifecycle_publisher(
            EmptyMsg, "/signal/ebs", LATCHED_QOS,
        )
        self._ebs_reset_pub = self.create_lifecycle_publisher(
            EmptyMsg, "/signal/ebs_reset", LATCHED_QOS,
        )

        # /odom infrastructure — created here, subscriptions and
        # timer come up in on_activate. Skipped entirely when
        # use_external_odometry_filter is True (odometry_filter_node
        # owns the equivalent surface in C++).
        if not self._use_external_filter:
            self._odom_filter = OdometryFilter()
            self._odom_pub = self.create_lifecycle_publisher(
                Odometry, "/odom", 50,
            )
            self._odom_tf_broadcaster = TransformBroadcaster(self)
            self._yaw_residual_pub = self.create_lifecycle_publisher(
                Float32, "/odom_diag/yaw_residual_rad_s", 10,
            )
            self._slip_flag_pub = self.create_lifecycle_publisher(
                Bool, "/odom_diag/slip_flag", 10,
            )
            self._effective_alpha_pub = self.create_lifecycle_publisher(
                Float32, "/odom_diag/effective_alpha_vx", 10,
            )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            "on_activate: starting /odom subscriptions + publish timer "
            f"({ODOM_PUBLISH_HZ:.0f} Hz)")

        # Reset the filter so a deactivate→activate cycle starts a
        # fresh stationary calibration. The car may have been moved
        # in sim during the inactive window; assuming continuity
        # would corrupt the bias estimates.
        if self._odom_filter is not None:
            self._odom_filter.reset()
        self._odom_first_publish_logged = False

        # Latched /signal/ebs_reset (post-#384). On the real car the
        # uDV firmware clears the EBS gate at power-up; in sim the
        # supervisor does the same the moment its lifecycle goes
        # active. Without this, the bridge's `ebs_triggered_` gate
        # stays latched from any previous run and every relayed
        # control command would be silently dropped. Pre-#384
        # control_node owned this publish on its own on_activate;
        # the topic is now exclusively supervisor-owned per the
        # diagram contract in docs/AUTONOMY.md.
        if self._ebs_reset_pub is not None:
            self._ebs_reset_pub.publish(EmptyMsg())

        # IMU / RPM / steering / brake subscriptions + publish timer
        # ALL skipped when the external C++ filter owns /odom — those
        # callbacks are exactly the 93 %-CPU Python hot path the port
        # was written to eliminate. The RuntimeControl feedback relay
        # and bridge publishers remain in either mode.
        if not self._use_external_filter:
            imu_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=2000,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._sub_imu = self.create_subscription(
                Imu, "/imu", self._on_imu, imu_qos,
                callback_group=self._cb_group,
            )
            rpm_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._sub_rpm = self.create_subscription(
                Float32, "/motor_rpm", self._on_rpm, rpm_qos,
                callback_group=self._cb_group,
            )
            self._sub_steering = self.create_subscription(
                Float32, "/steering_angle", self._on_steering, rpm_qos,
                callback_group=self._cb_group,
            )
            self._sub_brake = self.create_subscription(
                Float32, "/brake_pressure", self._on_brake, rpm_qos,
                callback_group=self._cb_group,
            )
            self._odom_pub_timer = self.create_timer(
                1.0 / ODOM_PUBLISH_HZ,
                self._publish_odom,
                callback_group=self._cb_group,
            )

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate: stopping /odom + subs")
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
        if self._runtime_feedback_sub is not None:
            self.destroy_subscription(self._runtime_feedback_sub)
            self._runtime_feedback_sub = None
        self._control_pub = None
        self._ebs_pub = None
        self._ebs_reset_pub = None
        # /odom infra — subs/timer already gone via on_deactivate, but
        # we still own the publisher + filter + broadcaster.
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

    # ------------------------------------------------------------------
    # /odom — IMU + RPM → dead-reckoning Odometry
    # ------------------------------------------------------------------
    def _on_imu(self, msg: Imu) -> None:
        """Drive the filter's predict step.

        Decimates the 400 Hz IMU stream by IMU_DECIMATION before
        pushing into the filter — see the module-level constant for
        the CPU rationale (#385). Bridge subscription stays at full
        depth so the unused samples flow through DDS at zero cost
        to us; we just skip the np.array construction + push_imu
        call for the dropped 3 of 4.
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
        """Cache the latest brake authority [0, 1]. Used inside
        push_rpm to scale α_vx during brake events (#383)."""
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

        # nav_msgs/Odometry: pose in header.frame_id (odom),
        # twist in child_frame_id (base_link). REP-103 axes.
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

        # odom → base_link TF (Phase 2 — #382). Same pose, broadcast
        # at the 100 Hz publish rate so downstream TF lookups see a
        # high-rate dead-reckoning leaf.
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

        # Phase 3 (#383) diagnostics — emit the filter's cross-check
        # residuals alongside /odom. Cheap (three Float32-ish topics
        # at 100 Hz); off by default in subscribers, only the tuning
        # plot panels open them.
        diag = self._odom_filter.diagnostics
        if self._yaw_residual_pub is not None:
            self._yaw_residual_pub.publish(Float32(data=float(diag.yaw_residual_rad_s)))
        if self._slip_flag_pub is not None:
            self._slip_flag_pub.publish(Bool(data=bool(diag.slip_flag)))
        if self._effective_alpha_pub is not None:
            self._effective_alpha_pub.publish(Float32(data=float(diag.effective_alpha_vx)))

    # ------------------------------------------------------------------
    # RuntimeControl feedback relay
    # ------------------------------------------------------------------
    def _relay_runtime_control_feedback(self, fb_msg) -> None:
        """Republish mission_control RuntimeControl feedback to the bridge."""
        fb = fb_msg.feedback
        if self._control_pub is not None:
            cmd = ControlCommand()
            cmd.header.stamp = fb.stamp
            cmd.throttle = float(fb.throttle)
            cmd.steering = float(fb.steering)
            cmd.brake = 0.0
            self._control_pub.publish(cmd)

        if fb.emergency and not self._runtime_ebs_latched:
            self.get_logger().warn(
                "RuntimeControl feedback emergency=true — "
                "publishing latched /signal/ebs")
            if self._ebs_pub is not None:
                self._ebs_pub.publish(EmptyMsg())
            self._runtime_ebs_latched = True


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
