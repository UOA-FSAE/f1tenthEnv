from random import random
from cmath import sqrt

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import ControlWorld
from ros_gz_interfaces.msg import WorldControl, WorldReset

import time

import message_filters


class Environment(Node):

    def __init__(self):
        super().__init__('environment')

        # Data Fields --------------------------------------------------------------------------------------------------
        self.lidar_data: LaserScan = LaserScan()
        self.imu_data: Imu = Imu()
        self.odom_data: Odometry = Odometry()
        self.pose = (0.0, 0.0)
        self.terminated_ = False
        self.reward_amount = 0.0

        # Data Subscriptions -------------------------------------------------------------------------------------------
        self.subscriber_lidar = message_filters.Subscriber(self, LaserScan, 'lidar')
        self.subscriber_imu = message_filters.Subscriber(self, Imu, 'imu')
        self.subscriber_navsat = message_filters.Subscriber(self, NavSatFix, 'navsat')  # Only works in sim
        self.subscriber_odom = message_filters.Subscriber(self, Odometry, 'model/f1tenth/odometry')

        self.subscriber_mf = message_filters.ApproximateTimeSynchronizer(
            [self.subscriber_lidar,
             self.subscriber_imu,
             self.subscriber_navsat,
             self.subscriber_odom],

            queue_size=1,
            slop=0.2
        )
        self.subscriber_mf.registerCallback(self.message_filter_callback)
        self.has_spun = False

        # Publishers ---------------------------------------------------------------------------------------------------
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Services -----------------------------------------------------------------------------------------------------
        self.reset_client = self.create_client(ControlWorld, '/world/car_world/control')

        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Control service not available, waiting again")

        self.reset_request = ControlWorld.Request()

        # Reward Stuff -------------------------------------------------------------------------------------------------
        self.time_since_start = time.monotonic()
        self.reward_pos = (0.0, 0.0)

        self.generate_reward()

        # Termination Stuff --------------------------------------------------------------------------------------------
        self.COLLISION_RANGE_: float = 0.115

    def spin_once(self):
        # This is a bad fix due to ros_gz_bridge node callback closing the message filter callback and so never
        # gets data.

        self.has_spun = False
        while not self.has_spun:
            rclpy.spin_once(self)

    def apply_action(self, linear_vel, angular_vel):
        # self.get_logger().info("Taking an action!")
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

    def step(self, action):

        # Take the Action
        lin_vel, ang_vel = action
        self.apply_action(lin_vel, ang_vel)

        # Process action for X time period and observe state

        # This specific implementation of waiting for X time period should be improved
        # rate = self.create_rate(20)

        # end_time = time.monotonic() + 0.1
        # while time.monotonic() < end_time:  # process data for X time frame

        # Observe the data
        self.spin_once()

        # Calculate Reward
        self.reward_amount = self.calculate_reward()

        # Check if termination conditions are met
        self.terminated_ = self.is_terminated()

        # if self.terminated_:
        #     break

        # Run at X Hz
        # rate.sleep()

        # Calculated the reward gained

        return self.get_observation()

    def is_terminated(self) -> bool:
        if self.terminated_:
            return True

        if min(self.lidar_data.ranges) <= self.COLLISION_RANGE_:
            return True

        # if time.monotonic() - self.time_since_start >= 10:
        #     self.reward_amount = -10.0
        #     return True

        return False

    def generate_reward(self):
        self.reward_pos = ((random() * 6) - 3, (random() * 6) - 3)

    def calculate_reward(self) -> float:
        if sqrt(
            pow(self.pose[0] - self.reward_pos[0], 2) +
            pow(self.pose[1] - self.reward_pos[1], 2)
        ).real <= 0.2:
            self.terminated_ = True
            return 100.0

        return -sqrt(
            pow(self.pose[0] - self.reward_pos[0], 2) +
            pow(self.pose[1] - self.reward_pos[1], 2)
        ).real

    def get_observation(self):
        # TODO: Add more to observation
        # Pose/Encoder = (x,y) | (log, lat)
        # Pose, Encoder, Velocity (IMU), LIDAR, Depth Camera
        # Return the step data: Observation, Reward, Terminated, Truncated, Info

        # recommended mapping of lidar is list(map(lambda x: x if not math.isinf(x) else -1, state[3].ranges.tolist()))
        return [self.pose, self.reward_pos, self.imu_data, self.lidar_data, self.odom_data], self.reward_amount, self.terminated_, None, None

    def reset(self):
        # Service call to reset the simulation
        # self.reward_timer.reset()
        # self.publisher_reward.publish(reward_msg)

        self.terminated_ = False
        self.reward_amount = 0.0
        self.generate_reward()

        # self.reset_request.world_control = WorldControl()
        #
        # world_reset = WorldReset()
        # world_reset.all = True
        #
        # self.reset_request.world_control.reset = world_reset
        #
        # # TODO: find out why the future isn't returning anything, even though it is successful
        # # self.reset_client.call_async(self.reset_request)
        # # self.spin_once()
        # # self.get_logger().info('PLEASE FINISH')
        # self.time_since_start = time.monotonic()

        return self.get_observation()

    # Callbacks
    def message_filter_callback(self, lidar_msg, imu_msg, navsat_msg: NavSatFix, odom_msg):
        self.has_spun = True
        self.lidar_data = lidar_msg
        self.imu_data = imu_msg
        self.pose = (navsat_msg.longitude * 1e5, navsat_msg.latitude * 1e5)
        self.odom_data = odom_msg
