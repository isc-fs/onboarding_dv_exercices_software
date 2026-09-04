"""
Bridge-only launch — starts ifssim_bridge without the autonomous pipeline.
Use this to verify connectivity and topic flow, or for manual/keyboard driving.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # LiDAR transport is UDP-only as of #322 (#321 marked TCP and UDS
    # soft-deprecated; this PR followed through on the deletion).
    # `LIDAR_TRANSPORT` env var and `lidar_transport` ROS parameter
    # are gone — the bridge node hard-wires UdpReceiver on port 51453.

    # LIDAR_VIZ_DECIMATION — opt-in subsampled LiDAR cloud for browser
    # visualisers. 0 (default) disables /lidar/Lidar1/viz; >=2 enables
    # it as every-Nth-point alongside the full cloud. The autonomy
    # stack always subscribes to /lidar/Lidar1 (full density).
    try:
        lidar_viz_decimation = int(os.environ.get("LIDAR_VIZ_DECIMATION", "0"))
    except ValueError:
        lidar_viz_decimation = 0

    return LaunchDescription([
        DeclareLaunchArgument('host',         default_value='host.docker.internal'),
        DeclareLaunchArgument('port',         default_value='41451'),
        DeclareLaunchArgument('mission_name', default_value='trackdrive'),
        DeclareLaunchArgument('track_name',   default_value='A'),

        Node(
            package='ifssim_bridge',
            executable='ifssim_bridge',
            name='ifssim_bridge',
            output='screen',
            parameters=[{
                'host':                  LaunchConfiguration('host'),
                'port':                  LaunchConfiguration('port'),
                'mission_name':          LaunchConfiguration('mission_name'),
                'track_name':            LaunchConfiguration('track_name'),
                'competition_mode':      False,
                'lidar_viz_decimation':  lidar_viz_decimation,
            }],
        ),

        # Foxglove Studio WebSocket bridge — connect at ws://localhost:8765
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                # send_buffer_limit caps the per-client WebSocket send
                # buffer. Default was 10 MB which holds ~6 PointCloud2
                # messages at the post-#255 1.53 MB/scan size; any
                # ~600 ms render hiccup on the Lichtblick side overflows
                # the buffer and foxglove_bridge drops messages — visible
                # as LiDAR flicker. 64 MB gives ~4 seconds of headroom,
                # well past any normal client-side stutter.
                'send_buffer_limit': 64 * 1024 * 1024,
                'use_sim_time': False,
                # See pipeline.launch.py for the full rationale on
                # topic_whitelist + use_compression. Same list here so
                # the bridge-only launch (replay / debug flows) gets
                # the same CPU-mitigated config.
                'topic_whitelist': [
                    "/Conos", "/Conos_Orange", "/Conos_raw",
                    "/Path", "/path_planning/debug",
                    "/slam/pose", "/cone_slam/gt_aligned",
                    "/cone_slam/gt_error_m",
                    "/control/v_set_mps", "/control/kappa_max_per_m",
                    "/ctrl/cmd_internal",
                    "/lidar/Lidar1/viz", "/imu", "/motor_rpm",
                    "/testing_only/odom", "/testing_only/track",
                    "/clicked_point", "/initialpose",
                    "/move_base_simple/goal", "/track_overlay",
                    "/tf", "/tf_static",
                ],
                'use_compression': True,
            }],
        ),
    ])
