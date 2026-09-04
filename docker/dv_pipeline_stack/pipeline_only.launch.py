"""
Pipeline-only launch — DV pipeline alignment series.

Brings up everything except the bridge + foxglove (those come from
bridge.launch.py running separately under entrypoint.sh):

  Always-active:
    (robot_state_publisher / joint_state_publisher were removed
    2026-05-11 along with the chassis URDF — pure-visualisation
    overhead. The autonomy's TF tree is map → odom → base_link
    plus the bridge's static sensor TFs rooted at base_link.)

  Mission management (LifecycleNodes, auto-configured+activated):
    mode_manager_node, mission_control_node, sim_supervisor_node

  Autonomy (LifecycleNodes, parked in `unconfigured` until
  mode_manager drives them through configure→activate via change_state
  fan-out triggered by mission_control_node.StartMission):
    cone_detection_node, slam_node, path_planning_node, control_node

The DV_DISABLE_CONTROL env switch from the pre-lifecycle layout has
been retired — leaving control_node parked in `unconfigured` is the
new way to keep it dormant. Once it's launched, mode_manager owns
whether it goes active or stays put.
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition

# ---------------------------------------------------------------------
# Topic remappings — same as pipeline.launch.py. See that file's
# header for the rationale.
# ---------------------------------------------------------------------
REMAP_LIDAR    = ("/fsds/lidar/Lidar1", "/lidar/Lidar1")
REMAP_GSS      = ("/fsds/gss",          "/gss")
REMAP_IMU      = ("/fsds/imu",          "/imu")
REMAP_GT       = ("/fsds/testing_only/odom", "/testing_only/odom")
REMAP_RPM      = ("/fsds/motor_rpm",    "/motor_rpm")
REMAP_CMD      = ("/fsds/control_command", "/control_command")
REMAP_STEERING = ("/fsds/steering_angle", "/steering_angle")
REMAP_BRAKE    = ("/fsds/brake_pressure", "/brake_pressure")


def _auto_active(package: str, executable: str, name: str,
                 remappings=None) -> list:
    """LifecycleNode + configure event + activate handler — auto-drives
    the node from unconfigured to active at launch start."""
    node = LifecycleNode(
        package=package,
        executable=executable,
        name=name,
        namespace="",
        output="screen",
        remappings=remappings or [],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node,
        goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))
    return [node, activate, configure]


def _autonomy_lifecycle(package: str, executable: str, name: str,
                        remappings=None) -> LifecycleNode:
    """LifecycleNode parked in `unconfigured` — mode_manager drives it."""
    return LifecycleNode(
        package=package,
        executable=executable,
        name=name,
        namespace="",
        output="screen",
        remappings=remappings or [],
    )


def generate_launch_description() -> LaunchDescription:
    actions = [
        DeclareLaunchArgument("host",         default_value="host.docker.internal"),
        DeclareLaunchArgument("port",         default_value="41451"),
        DeclareLaunchArgument("mission_name", default_value="trackdrive"),
        DeclareLaunchArgument("track_name",   default_value="A"),
    ]

    # Mission management (auto-active) — see pipeline.launch.py for
    # the order rationale.
    actions += _auto_active("mode_manager", "mode_manager_node", "mode_manager_node")
    actions += _auto_active("mission_control", "mission_control_node", "mission_control_node")
    # sim_supervisor needs /imu + /motor_rpm remapped onto /fsds/* so
    # its OdometryFilter sees the bridge's sensor stream.
    actions += _auto_active(
        "sim_supervisor", "sim_supervisor_node", "sim_supervisor_node",
        # See pipeline.launch.py for the REMAP_CMD rationale (#384)
        # and the IMU-remap-still-here rationale (#432 Phase 2 — the
        # supervisor's `use_external_odometry_filter` parameter
        # defaults true; remaps stay for the override-to-Python case).
        remappings=[REMAP_IMU, REMAP_RPM, REMAP_STEERING, REMAP_BRAKE,
                    REMAP_CMD],
    )
    # Autonomy lifecycle nodes (unconfigured until mode_manager)
    actions.append(_autonomy_lifecycle(
        "odometry_filter_node", "odometry_filter_node",
        "odometry_filter_node",
        remappings=[REMAP_IMU, REMAP_RPM, REMAP_STEERING, REMAP_BRAKE],
    ))
    actions.append(_autonomy_lifecycle(
        "cone_detection", "cone_detection_node", "cone_detection_node",
        remappings=[REMAP_LIDAR],
    ))
    actions.append(_autonomy_lifecycle(
        "cone_slam", "slam_node", "slam_node",
        remappings=[REMAP_IMU, REMAP_RPM, REMAP_GT],
    ))
    actions.append(_autonomy_lifecycle(
        "path_planning", "path_planning_node", "path_planning_node",
    ))
    actions.append(_autonomy_lifecycle(
        "control", "control_node", "control_node",
        # Post-#384: control_node publishes /ctrl/cmd_internal, not
        # /fsds/control_command. See pipeline.launch.py.
        remappings=[],
    ))

    return LaunchDescription(actions)
