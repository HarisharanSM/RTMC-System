"""Headless integrated simulator demo; RViz is an explicit desktop option."""

from pathlib import Path
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start_demo(context):
    binary = Path(LaunchConfiguration('simulator_binary').perform(context)).expanduser().resolve()
    assets = Path(LaunchConfiguration('assets').perform(context)).expanduser().resolve()
    if not binary.is_file() or not assets.is_dir():
        raise RuntimeError('Pass simulator_binary:=/absolute/path/build/pcan_demo and assets:=/absolute/path/repo')
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            listener.bind(('127.0.0.1', 8082))
        except OSError as exc:
            raise RuntimeError('Port 8082 is occupied; stop the existing simulator before this demo') from exc
    simulator = ExecuteProcess(cmd=[str(binary), '--assets', str(assets),
                                   '--command-source', 'ros2'], output='screen')
    bridge = Node(package='rtmc_ros2', executable='bridge', output='screen',
                  parameters=[{'enable_commands': ParameterValue(
                      LaunchConfiguration('enable_commands'), value_type=bool)}])
    description = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('rtmc_description')) / 'launch' / 'description.launch.py')),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items())
    # Shut down only this launch's children when either control-side process exits.
    return [RegisterEventHandler(OnProcessExit(target_action=process,
                on_exit=[EmitEvent(event=Shutdown(reason='demo process exited'))]))
            for process in (simulator, bridge)] + [simulator, bridge, description]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulator_binary', description='Absolute path to pcan_demo'),
        DeclareLaunchArgument('assets', description='Absolute repository path for runtime assets'),
        DeclareLaunchArgument('enable_commands', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        OpaqueFunction(function=start_demo),
    ])
