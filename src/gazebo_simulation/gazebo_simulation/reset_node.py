import os
import uuid

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription

from std_msgs.msg import Bool


class ResetNode(Node):

    def __init__(self):
        super().__init__('reset_node')
        self.current_vehicle_id = 'f1tenth'

        self.subcriber_reset: Subscription = self.create_subscription(
            Bool,
            'reset',
            self.reset_callback,
            10
        )

    def reset_callback(self, sub_msg: Bool):

        if sub_msg.data:
            os.system("gz service -s /world/car_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout "
                      f"1000 --req 'name:\"{self.current_vehicle_id}\", type:2'")

            self.current_vehicle_id = str(uuid.uuid4())

            os.system("gz service -s /world/car_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean "
                      f"--timeout 1000 --req 'sdf_filename: \"f1tenth_track_sdf/car.sdf\", name: \"{self.current_vehicle_id}\"'")


def main(args=None):
    rclpy.init(args=args)

    reset_node = ResetNode()

    rclpy.spin(reset_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reset_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
