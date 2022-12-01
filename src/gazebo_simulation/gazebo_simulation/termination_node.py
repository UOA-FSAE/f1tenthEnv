import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32


class TerminationNode(Node):

    def __init__(self):
        super().__init__('termination_node')

        self.COLLISION_RANGE_ = 0.115
        self.NEGATIVE_REWARD_ = -10

        self.reward_timer = self.create_timer(
            10,  # in seconds
            self.timer_callback
        )

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
        self.publisher_reward: Publisher = self.create_publisher(
            Int32,
            'reward',
            10
        )

        self.subscriber_lidar: Subscription = self.create_subscription(
            LaserScan,
            'lidar',
            self.lidar_callback,
            10
        )
        self.subscriber_reward: Subscription = self.create_subscription(
            Int32,
            'reward',
            self.reward_callback,
            10
        )

    def lidar_callback(self, sub_msg: LaserScan) -> None:
        if min(sub_msg.ranges) <= self.COLLISION_RANGE_:
            self.terminate_sim()

    def reward_callback(self, sub_msg: Int32) -> None:
        if sub_msg != 0:
            self.reward_timer.reset()

    def timer_callback(self) -> None:
        self.terminate_sim()

    def terminate_sim(self):
        reset_msg = Bool()
        reset_msg.data = True

        reward_msg = Int32()
        reward_msg.data = self.NEGATIVE_REWARD_

        self.reward_timer.reset()
        self.publisher_reward.publish(reward_msg)

        self.publisher_reset.publish(reset_msg)
        self.publisher_termination.publish(reset_msg)


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
