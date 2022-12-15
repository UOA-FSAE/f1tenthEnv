import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

gz_to_ros_message_type = {
    'gz.msgs.LaserScan': 'sensor_msgs/msg/LaserScan',
    'gz.msgs.Twist': 'geometry_msgs/msg/Twist',
    'gz.msgs.IMU': 'sensor_msgs/msg/Imu',
    'gz.msgs.NavSat': 'sensor_msgs/msg/NavSatFix',
    'gz.msgs.Boolean': 'std_msgs/msg/Bool',
}

gz_launch_file = 'f1tenth_track_sdf/empty.sdf'
hz = '1000'


def generate_launch_description():
    pkg_traxxas_4x4_zoomer_bringup = get_package_share_directory('traxxas_4x4_zoomer_bringup')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_traxxas_4x4_zoomer_bringup, 'launch', 'start_simulation.launch.py'))
    )

    training_node = Node(
        package='rl_control',
        executable='training_node'
    )

    return LaunchDescription([
        sim,
        training_node
    ])
