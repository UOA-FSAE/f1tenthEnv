import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32
from ros_gz_interfaces.srv import ControlWorld
from ros_gz_interfaces.msg import WorldControl, WorldReset


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
        self.subscriber_reset: Subscription = self.create_subscription(
            Bool,
            'reset',
            self.reset_callback,
            10
        )

        # Resetting Simulation
        self.client = self.create_client(ControlWorld, '/world/car_world/control')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Control service not available, waiting again")

        self.request = ControlWorld.Request()

    def lidar_callback(self, sub_msg: LaserScan) -> None:
        if min(sub_msg.ranges) <= self.COLLISION_RANGE_:
            self.terminate_sim()

    def reward_callback(self, sub_msg: Int32) -> None:
        if sub_msg.data != 0:
            self.reward_timer.reset()

    def reset_callback(self, sub_msg: Bool) -> None:
        self.get_logger().info("Reset Triggered!")
        if sub_msg.data:
            self.terminate_sim()

    def timer_callback(self) -> None:
        self.terminate_sim()

    def terminate_sim(self):
        self.get_logger().info("Simulation Restarting...")

        reward_msg = Int32()
        reward_msg.data = self.NEGATIVE_REWARD_

        self.reward_timer.reset()
        self.publisher_reward.publish(reward_msg)

        self.request.world_control = WorldControl()

        world_reset = WorldReset()
        world_reset.all = True

        self.request.world_control.reset = world_reset

        # TODO: find out why the future isn't returning anything, even though it is successful
        self.client.call_async(self.request)

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
