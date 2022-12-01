import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Vector3, Twist

import message_filters


class AgentEnvNode(Node):

    def __init__(self):
        super().__init__('agent_env_node')

        # -------------------------- Data Fields ------------------------------------------------- #
        self.lidar_data: list[float] = LaserScan().ranges
        self.imu_data: tuple[list[float], list[float], list[float]] = \
            Imu().orientation, Imu().angular_velocity, Imu().linear_acceleration
        self.reward_data: int = Int32().data
        self.termination_data: bool = Bool().data

        # -------------------------- Subscribe topics -------------------------------------------- #
        self.subcriber_termination = self.create_subscription(
            Bool,
            'termination',
            self.set_termination_callback,
            10
        )

        self.subcriber_lidar = message_filters.Subscriber(self, LaserScan, 'lidar')
        self.subcriber_imu = message_filters.Subscriber(self, Imu, 'imu')
        self.subcriber_reward = message_filters.Subscriber(self, Int32, 'reward')

        self.subcriber_mf = message_filters.ApproximateTimeSynchronizer(
            [self.subcriber_lidar,
             self.subcriber_imu,
             self.subcriber_reward],

            queue_size=10,
            slop=0.1
        )
        self.subcriber_mf.registerCallback(self.message_filter_callback)

        # -------------------------- Publish topics ---------------------------------------------- #
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def set_termination_callback(self, termination_msg: Bool):
        if not self.termination_data:
            self.termination_data = termination_msg.data

    def message_filter_callback(self, lidar_msg: LaserScan, imu_msg: Imu, reward_msg: Int32):
        self.lidar_data = lidar_msg.ranges
        self.imu_data = imu_msg.orientation, imu_msg.angular_velocity, imu_msg.linear_acceleration
        # make sure not to override reward data since reward gets updated faster than its used
        self.reward_data = reward_msg.data if self.reward_data == 0 else self.reward_data

    def observation_request(self) -> tuple[list[float], tuple[list[float], list[float], list[float]], int, bool]:
        return self.lidar_data, self.imu_data, self.reward_data, self.termination_data

    def action_request(self, linear: float, angle: float):
        twist_msg: Twist = Twist()

        vec3_linear: Vector3 = Vector3()
        vec3_linear.x = linear
        twist_msg.linear = vec3_linear

        vec3_angle: Vector3 = Vector3()
        vec3_angle.z = angle
        twist_msg.angular = vec3_angle

        self.publisher_cmd_vel.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    agent_env_node = AgentEnvNode()

    rclpy.spin(agent_env_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    agent_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
