import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_simulation = get_package_share_directory('gazebo_simulation')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_simulation, 'start_simulation.launch.py')),
    )

    data_env_service = Node(
        package="driver",
        executable="env_data_service_node"
    )

    return LaunchDescription([
        gz_sim,
        data_env_service
    ])
