import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_gazebo_simulation = get_package_share_directory('gazebo_simulation')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_simulation, 'launch', 'start_simulation.launch.py')),
    )

    return LaunchDescription([
        gz_sim
    ])
