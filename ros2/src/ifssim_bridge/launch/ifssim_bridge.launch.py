from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost',
                              description='IFSSIM simulator host'),
        DeclareLaunchArgument('port', default_value='41451',
                              description='IFSSIM RPC server port'),
        DeclareLaunchArgument('timeout', default_value='5.0',
                              description='Connection timeout in seconds'),
        DeclareLaunchArgument('mission_name', default_value='trackdrive',
                              description='Mission identifier'),
        DeclareLaunchArgument('track_name', default_value='A',
                              description='Track identifier'),
        DeclareLaunchArgument('competition_mode', default_value='false',
                              description='Enable competition mode (disables testing topics)'),

        Node(
            package='ifssim_bridge',
            executable='ifssim_bridge',
            name='ifssim_bridge',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'timeout': LaunchConfiguration('timeout'),
                'mission_name': LaunchConfiguration('mission_name'),
                'track_name': LaunchConfiguration('track_name'),
                'competition_mode': LaunchConfiguration('competition_mode'),
            }]
        ),
    ])
