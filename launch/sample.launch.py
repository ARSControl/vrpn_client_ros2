import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    parameters_file_path = Path(get_package_share_directory('vrpn_client_ros2'), 'config', 'param.yaml')
    return launch.LaunchDescription([
            launch_ros.actions.Node(
                package="vrpn_client_ros2",
                executable="vrpn_client_node",
                output="screen",
                parameters=[parameters_file_path]
            )
    ])