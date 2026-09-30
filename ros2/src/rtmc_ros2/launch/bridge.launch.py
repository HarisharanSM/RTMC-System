from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_commands', default_value='false'),
        DeclareLaunchArgument('base_url', default_value='http://127.0.0.1:8082'),
        Node(package='rtmc_ros2', executable='bridge', name='rtmc_bridge', output='screen',
             parameters=[{'enable_commands': LaunchConfiguration('enable_commands'),
                          'base_url': LaunchConfiguration('base_url')}]),
    ])
