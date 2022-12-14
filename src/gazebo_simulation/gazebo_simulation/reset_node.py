import os
import uuid

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription

from std_msgs.msg import Bool
from ros_gz_interfaces.srv import ControlWorld
from ros_gz_interfaces.msg import WorldControl, WorldReset


class ResetNode(Node):

    def __init__(self):
        super().__init__('reset_node')

        self.client = self.create_client(ControlWorld, '/world/car_world/control')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Control service not available, waiting again")

        self.request = ControlWorld.Request()
        self.current_vehicle_id = 'f1tenth'

        self.subscriber_reset: Subscription = self.create_subscription(
            Bool,
            'reset',
            self.reset_callback,
            10
        )

    def reset_callback(self, sub_msg: Bool):
        self.get_logger().info("Reset Topic Triggered!")
        if sub_msg.data:
            self.send_reset_request()

    def send_reset_request(self) -> ControlWorld:

        self.request.world_control = WorldControl()
        world_reset = WorldReset()
        world_reset.all = True

        self.request.world_control.reset = world_reset

        future = self.client.call_async(self.request)
        self.get_logger().info("Entering Spin")
        rclpy.spin_until_future_complete(self, future, timeout_sec=1)
        self.get_logger().info("Exiting Spin")
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    reset_node = ResetNode()

    reset_node.get_logger().info("Spinning Reset Node...")
    rclpy.spin_once(reset_node)
    rclpy.spin_once(reset_node)
    rclpy.spin_once(reset_node)
    reset_node.get_logger().info("Reset Node Stopped Spinning...")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reset_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
