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

gz_launch_file = 'f1tenth_track_sdf/f1tenth_world.sdf'
hz = '1000'


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # noinspection PyTypeChecker
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),

        launch_arguments={
            'gz_args': f'-r {gz_launch_file} -z {hz}'
        }.items(),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'/lidar@{gz_to_ros_message_type["gz.msgs.LaserScan"]}@gz.msgs.LaserScan',
            f'/cmd_vel@{gz_to_ros_message_type["gz.msgs.Twist"]}@gz.msgs.Twist',
            f'/imu@{gz_to_ros_message_type["gz.msgs.IMU"]}@gz.msgs.IMU',
            f'/navsat@{gz_to_ros_message_type["gz.msgs.NavSat"]}@gz.msgs.NavSat',
            f'/reset@{gz_to_ros_message_type["gz.msgs.Boolean"]}@gz.msgs.Boolean',
        ]

    )

    reward_gate_node = Node(
        package='gazebo_simulation',
        executable='reward_gate_node'
    )

    termination_node = Node(
        package='gazebo_simulation',
        executable='termination_node'
    )

    reset_node = Node(
        package='gazebo_simulation',
        executable='reset_node',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        gz_bridge,
        reward_gate_node,
        termination_node,
        reset_node
    ])
