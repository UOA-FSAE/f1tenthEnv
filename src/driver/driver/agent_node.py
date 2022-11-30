import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu


class AgentNode(Node):

    def __init__(self):
        super().__init__('agent_node')

        # subs and pubs
        self.subscriber_lidar: Subscription = self.create_subscription(
            LaserScan,
            'lidar',
            self.listener_callback,
            10
        )
        self.subscriber_imu: Subscription = self.create_subscription(
            Imu,
            'imu',
            self.listener_callback,
            10
        )
        self.subscriber_reward: Subscription = self.create_subscription(
            Bool,
            'reward',
            self.listener_callback,
            10
        )
        self.subscriber_termination: Subscription = self.create_subscription(
            Bool,
            'termination',
            self.listener_callback,
            10
        )

        self.publisher_cmd_vel: Publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def listener_callback(self, sub_msg) -> None:
        pass


def main(args=None):
    rclpy.init(args=args)

    agent_node = AgentNode()

    rclpy.spin(agent_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    agent_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
