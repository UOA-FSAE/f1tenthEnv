import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import LaserScan, Imu

import message_filters


class EnvDataServiceNode(Node):

    def __init__(self):
        super().__init__('env_data_service_node')

        # -------------------------- Data Fields ------------------------------------------------- #
        self.lidar_data = LaserScan()
        self.imu_data = Imu()
        self.reward_data = Int32()
        self.termination_data = Bool()

        # -------------------------- Service ----------------------------------------------------- #
        self.srv = self.create_service(
            # TODO: Create srv for return lidar, imu, reward, and termination data
            'get_env_data',
            self.get_data_service
        )

        # -------------------------- Subscribe topics -------------------------------------------- #
        self.subscriber_termination = self.create_subscription(
            Bool,
            'termination',
            self.termination_callback,
            10
        )
        self.subscriber_reward = self.create_subscription(
            Int32,
            'reward',
            self.reward_callback,
            10
        )

        self.subscriber_lidar = message_filters.Subscriber(self, LaserScan, 'lidar')
        self.subscriber_imu = message_filters.Subscriber(self, Imu, 'imu')

        self.subscriber_mf = message_filters.ApproximateTimeSynchronizer(
            [self.subscriber_lidar,
             self.subscriber_imu],

            queue_size=10,
            slop=0.1
        )
        self.subscriber_mf.registerCallback(self.message_filter_callback)

    def get_data_service(self, request, response):
        pass

    def message_filter_callback(self, lidar_msg, imu_msg):
        pass

    def reward_callback(self, reward_msg):
        pass

    def termination_callback(self, termination_msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    env_data_service_node = EnvDataServiceNode()

    rclpy.spin(env_data_service_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    env_data_service_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
