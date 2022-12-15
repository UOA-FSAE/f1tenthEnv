import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from geometry_msgs.msg import Vector3, Twist
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
        self.navsat_data: NavSatFix = NavSatFix()  # Only works in sim

        # Data Subscriptions -------------------------------------------------------------------------------------------
        self.subscriber_lidar = message_filters.Subscriber(self, LaserScan, 'lidar')
        self.subscriber_imu = message_filters.Subscriber(self, Imu, 'imu')
        self.subscriber_navsat = message_filters.Subscriber(self, NavSatFix, 'navsat')  # Only works in sim

        self.subscriber_mf = message_filters.ApproximateTimeSynchronizer(
            [self.subscriber_lidar,
             self.subscriber_imu,
             self.subscriber_navsat],

            queue_size=1,
            slop=0.1
        )
        self.subscriber_mf.registerCallback(self.message_filter_callback)

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
        # TODO: Note that reward gates won't work on real robot, so eventually remove

        self.curr_reward_gate: int = 0
        self.POSITIVE_REWARD: int = 10
        self.last_reward_time = time.monotonic()

        self.reward_gates: dict[int: (float, float, float, float)] = {
            0: (-1.45e-05, 1.27e-05, 2.00e-05, 3.00e-05),
            1: (-1.45e-05, 1.27e-05, 7.00e-05, 8.00e-05),
            2: (-1.45e-05, 1.27e-05, 0.00015, 0.00016),
            3: (-2.54e-05, -1.84e-05, 0.000169, 0.000196),
            4: (-5.80E-05, -3.19E-05, 0.000156, 0.000161),
            5: (-0.0001, -9.37E-05, 9.33E-05, 0.000116),
            6: (-0.000131, -0.000106, 0.000148, 0.000156),
            7: (-0.000151, -0.000142, 0.000178, 0.000204),
            8: (-0.000232, -0.000207, 9.70E-05, 0.000107),
            9: (-0.00023, -0.000205, 2.79E-05, 3.59E-05),
            10: (-0.000227, -0.000203, -2.19E-05, -1.49E-05),
            11: (-0.000197, -0.000192, -5.00E-05, -2.72E-05),
            12: (-0.000123, -0.000117, -1.47E-05, 1.05E-05),
            13: (-8.46E-05, -7.43E-05, -1.49E-05, 1.05E-05),
            14: (-2.50E-05, -1.76E-05, -5.23E-05, -2.78E-05),
        }

        # Termination Stuff --------------------------------------------------------------------------------------------
        self.COLLISION_RANGE_: float = 0.115

    def apply_action(self, linear_vel, angular_vel):
        self.get_logger().info("Taking an action!")
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

        reward = 0
        terminated = False

        # Process action for X time period and observe state

        # This specific implementation of waiting for X time period should be improved
        rate = self.create_rate(20)

        end_time = time.monotonic() + 1
        while time.monotonic() < end_time:  # process data for X time frame

            # Observe the data
            rclpy.spin_once(self)  # this triggers callbacks to check if we need to reset etc

            # Check if termination conditions are met
            terminated = self.is_terminated()

            # Calculate Reward
            # TODO: decide whether to have immediate or cumulative reward
            reward = self.calculate_reward()

            if terminated:
                break

            # Run at X Hz
            rate.sleep()

        # Calculated the reward gained

        return self.get_observation(reward=reward, terminated=terminated)

    def is_terminated(self) -> bool:
        if min(self.lidar_data.ranges) <= self.COLLISION_RANGE_:
            return True

        if self.last_reward_time - time.monotonic() >= 10:
            return True

        return False

    def calculate_reward(self) -> int:
        lat_start, lat_end, log_start, log_end = self.reward_gates[self.curr_reward_gate]

        reward = 0
        if (lat_start <= self.navsat_data.latitude <= lat_end) and (log_start <= self.navsat_data.longitude <= log_end):
            reward += 10

        return reward

    def get_observation(self, reward=0, terminated=False):
        # TODO: Add more to observation
        # Pose, Encoder, Velocity (IMU), LIDAR, Depth Camera
        # Return the step data: Observation, Reward, Terminated, Truncated, Info
        return [self.lidar_data, self.imu_data], reward, terminated, None, None

    def reset(self):
        # Service call to reset the simulation
        # self.reward_timer.reset()
        # self.publisher_reward.publish(reward_msg)

        self.reset_request.world_control = WorldControl()

        world_reset = WorldReset()
        world_reset.all = True

        self.reset_request.world_control.reset = world_reset

        # TODO: find out why the future isn't returning anything, even though it is successful
        self.reset_client.call(self.reset_request)

        rclpy.spin_once(self)  # this triggers callbacks to check if we need to reset etc

        return self.get_observation()

    # Callbacks
    def message_filter_callback(self, lidar_msg, imu_msg, navsat_msg):
        self.lidar_data = lidar_msg
        self.imu_data = imu_msg
        self.navsat_data = navsat_msg

    # def reward_callback(self, reward_msg):
    #     self.reward_data = reward_msg
    #
    # def termination_callback(self, termination_msg):
    #     self.termination_data = termination_msg

    # def navsat_listener_callback(self, sub_msg: NavSatFix) -> None:
    #     lat_start, lat_end, log_start, log_end = self.reward_gates[self.curr_reward_gate]
    #     if (lat_start <= sub_msg.latitude <= lat_end) and (log_start <= sub_msg.longitude <= log_end):
    #         pub_msg = Int32()
    #         pub_msg.data = 10
    #
    #         self.curr_reward_gate = (self.curr_reward_gate + 1) % 15
    #         self.publisher_reward.publish(pub_msg)
    #     else:
    #         pub_msg = Int32()
    #         pub_msg.data = 0
    #         self.publisher_reward.publish(pub_msg)
