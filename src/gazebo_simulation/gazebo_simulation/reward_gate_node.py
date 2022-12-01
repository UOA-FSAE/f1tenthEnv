""" ROS2 node for giving rewards for reward gates during simulation

This node will read the NavSatFix and check that if it has entered a reward gate it will return
a reward.

"""

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix


class RewardGateNode(Node):

    def __init__(self):
        super().__init__('reward_gate_node')

        self.curr_reward_gate: int = 0

        # reward gate number: lat_start lat_end log_start log_end
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

        self.publisher_reward: Publisher = self.create_publisher(
            Int32,
            'reward',
            10
        )

        self.subscriber_navsat: Subscription = self.create_subscription(
            NavSatFix,
            'navsat',
            self.listener_callback,
            10
        )

    def listener_callback(self, sub_msg: NavSatFix) -> None:
        lat_start, lat_end, log_start, log_end = self.reward_gates[self.curr_reward_gate]
        if (lat_start <= sub_msg.latitude <= lat_end) and (log_start <= sub_msg.longitude <= log_end):
            pub_msg = Int32()
            pub_msg.data = 10

            self.curr_reward_gate = (self.curr_reward_gate + 1) % 15
            self.publisher_reward.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)

    reward_gate_node = RewardGateNode()

    rclpy.spin(reward_gate_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reward_gate_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
