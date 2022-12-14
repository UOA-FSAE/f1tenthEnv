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
    'gz.msgs.Odometry': 'nav_msgs/msg/Odometry',
    'gz.msgs.Pose': 'geometry_msgs/msg/PoseStamped'
}

gz_launch_file = 'f1tenth_track_sdf/empty.sdf'
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
            f'/model/f1tenth/odometry@{gz_to_ros_message_type["gz.msgs.Odometry"]}@gz.msgs.Odometry',
            f'/model/f1tenth/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            f'/world/car_world/control@ros_gz_interfaces/srv/ControlWorld'
        ],
        remappings=[
            ('/model/f1tenth/tf', '/tf')
        ]
    )

    return LaunchDescription([
        gz_sim,
        gz_bridge
    ])
