import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import LaserScan, Imu
from service_interface.srv import VehicleEnvData

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
            VehicleEnvData,
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
        print('running service')
        response.lidar = self.lidar_data
        print('sent lidar data')
        response.imu = self.imu_data
        print('sent imu data')
        response.reward = self.reward_data
        print('sent reward data')
        response.termination = self.termination_data
        print('sent termination data')
        print('ending service')

        return response

    def message_filter_callback(self, lidar_msg, imu_msg):
        self.lidar_data = lidar_msg
        self.imu_data = imu_msg

    def reward_callback(self, reward_msg):
        if self.reward_data.data == 0:
            self.reward_data = reward_msg

    def termination_callback(self, termination_msg):
        if not self.termination_data.data:
            self.termination_data = termination_msg


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
