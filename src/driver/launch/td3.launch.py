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

    # td3_node = Node(
    #     package="driver",
    #     executable="TD3",
    #     output='screen'
    # )

    environment = Node(
        package="driver",
        executable="training_node",
        output="screen"
    )
    # data_env_service = Node(
    #     package="driver",
    #     executable="env_data_service_node"
    # )

    return LaunchDescription([
        gz_sim,
        environment
        # data_env_service,
    ])
