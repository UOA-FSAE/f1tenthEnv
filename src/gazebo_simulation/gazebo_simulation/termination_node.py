import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class TerminationNode(Node):

    def __init__(self):
        super().__init__('termination_node')
        self.publisher_reset: Publisher = self.create_publisher(
            Bool,
            'reset',
            10
        )
        self.publisher_termination: Publisher = self.create_publisher(
            Bool,
            'termination',
            10
        )

        self.subcriber_navsat: Subscription = self.create_subscription(
            LaserScan,
            'lidar',
            self.listener_callback,
            10
        )

    def listener_callback(self, sub_msg: LaserScan) -> None:
        pub_msg = Bool()

        pub_msg.data = min(sub_msg.ranges) <= 0.115

        self.publisher_reset.publish(pub_msg)
        self.publisher_termination.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)

    termination_node = TerminationNode()

    rclpy.spin(termination_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    termination_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
