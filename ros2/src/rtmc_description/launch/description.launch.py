from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    share = Path(get_package_share_directory('rtmc_description'))
    description = (share / 'urdf' / 'rtmc.urdf').read_text(encoding='utf-8')
    nodes = [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='rtmc_robot_state_publisher', output='screen',
        parameters=[{'robot_description': ParameterValue(description, value_type=str)}],
        remappings=[('joint_states', '/rtmc/joint_states')],
    )]
    if LaunchConfiguration('use_rviz').perform(context).lower() in ('true', '1', 'yes'):
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rtmc_rviz', output='screen',
            arguments=['-d', str(share / 'rviz' / 'rtmc.rviz')],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Start optional RViz visualization'),
        OpaqueFunction(function=_nodes),
    ])
