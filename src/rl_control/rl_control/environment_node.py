import time
from cmath import sqrt
from random import random

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration

import message_filters

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3, Twist, TransformStamped
from ros_gz_interfaces.msg import WorldControl, WorldReset

from ros_gz_interfaces.srv import ControlWorld

from tf2_ros.buffer import Buffer as TF_Buffer
from tf2_ros import TransformListener


class Environment(Node):
    def __init__(self, step_duration: float = 1.0, boundary_size: float = 10):
        super().__init__('environment')

        # config fields
        self.step_duration: float = step_duration
        self.boundary_size: float = boundary_size
        self.spin_flag: bool = False

        # Data fields
        self.lidar_data: LaserScan = LaserScan()
        self.imu_data: Imu = Imu()
        self.odom_data: Odometry = Odometry()
        self.tf_data: TransformStamped = TransformStamped()
        self.reward_tf_data: TransformStamped = TransformStamped()

        self.reset_request = ControlWorld.Request()

        self.reward_: float = 0.0
        self.terminated_: bool = False
        self.has_spun: bool = False

        # Data Subscriptions
        self.subscriber_lidar = message_filters.Subscriber(self, LaserScan, 'lidar')
        self.subscriber_imu = message_filters.Subscriber(self, Imu, 'imu')
        self.subscriber_odom = message_filters.Subscriber(self, Odometry, 'model/f1tenth/odometry')

        self.subscriber_mf = message_filters.ApproximateTimeSynchronizer(
            [self.subscriber_lidar,
             self.subscriber_imu,
             self.subscriber_odom],

            queue_size=1,
            slop=0.2
        )
        self.subscriber_mf.registerCallback(self.message_filter_callback)

        # TF buffer
        self.tf_buffer = TF_Buffer()
        TransformListener(self.tf_buffer, self, spin_thread=True)

        # Services
        self.reset_client = self.create_client(ControlWorld, '/world/car_world/control')

        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Control service not available, waiting again")

        # Data Publishers
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    # Public functions -------------------------------------------------------------------------------------------------
    def step(self, action: tuple[float, float]):
        """
        Use this function to take a step in the simulation.

        Args:
            action: target linear velocity and angle

        Returns:
            Obs, reward, terminated, truncated, info
        """
        self.set_action(*action)

        rate = self.create_rate(20)
        end_step_time = time.monotonic() + self.step_duration
        while time.monotonic() < end_step_time:

            self.tf_data = self.tf_buffer.lookup_transform('f1tenth/odom', 'f1tenth/chassis',
                                                           time=rclpy.time.Time(), timeout=Duration(seconds=5))

            self.terminated_ = self.get_is_terminated()
            self.reward_ = self.get_reward()

            if self.terminated_:
                break
            rate.sleep()

        return self.get_observation()

    def reset(self):
        """
        This function resets the world

        Note: this function will take a while to reset the env after you make an action and your guess is as good as
        mine as to why this happens


        Returns:
            Obs, reward, terminated, truncated, info
        """
        #
        # self.reset_request.world_control = WorldControl()
        #
        # world_reset = WorldReset()
        # world_reset.all = True
        #
        # self.reset_request.world_control.reset = world_reset
        #
        # self.reset_client.call_async(self.reset_request)

        self.tf_data = self.tf_buffer.lookup_transform('f1tenth/odom', 'f1tenth/chassis',
                                                       time=rclpy.time.Time(), timeout=Duration(seconds=5))

        self.set_reward()
        return self.get_observation()

    # Get functions ----------------------------------------------------------------------------------------------------
    def get_observation(self):
        return [self.tf_data, self.reward_tf_data, self.imu_data, self.lidar_data, self.odom_data], \
            self.reward_, self.terminated_, None, None

    def get_reward(self) -> float:
        if sqrt(
            pow(self.tf_data.transform.translation.x - self.reward_tf_data.transform.translation.x
                , 2) +
            pow(self.tf_data.transform.translation.y - self.reward_tf_data.transform.translation.y
                , 2)
        ).real <= 0.2:
            self.terminated_ = True
            return 100.0

        if self.terminated_:
            return -100.0

        return -sqrt(
            pow(self.tf_data.transform.translation.x - self.reward_tf_data.transform.translation.x
                , 2) +
            pow(self.tf_data.transform.translation.y - self.reward_tf_data.transform.translation.y
                , 2)
        ).real

    def get_is_terminated(self) -> bool:
        # terminates if out of bounds
        # if self.tf_data.transform.translation.x > self.boundary_size or \
        #         self.tf_data.transform.translation.y > self.boundary_size or \
        #         self.tf_data.transform.translation.x < -self.boundary_size or \
        #         self.tf_data.transform.translation.y < -self.boundary_size:
        #     return True

        return False

    # Set functions ----------------------------------------------------------------------------------------------------
    def set_action(self, linear_vel: float, angular_vel: float):
        twist_msg: Twist = Twist()

        vec3_linear: Vector3 = Vector3()
        vec3_linear.x = linear_vel
        vec3_linear.y = 0.0
        vec3_linear.z = 0.0
        twist_msg.linear = vec3_linear

        vec3_angle: Vector3 = Vector3()
        vec3_angle.x = 0.0
        vec3_angle.y = 0.0
        vec3_angle.z = angular_vel
        twist_msg.angular = vec3_angle

        self.publisher_cmd_vel.publish(twist_msg)

    def set_reward(self):
        self.reward_tf_data = self.tf_data
        self.reward_tf_data.transform.translation.x = random() * 20 - 10
        self.reward_tf_data.transform.translation.y = random() * 20 - 10

    # Callbacks --------------------------------------------------------------------------------------------------------
    def message_filter_callback(self, lidar_msg: LaserScan, imu_msg: Imu, odom_msg: Odometry):
        self.spin_flag = True

        self.lidar_data = lidar_msg
        self.imu_data = imu_msg
        self.odom_data = odom_msg
