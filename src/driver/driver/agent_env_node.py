import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Vector3, Twist

from service_interface.src import VehicleEnvData


class AgentEnvNode(Node):

    def __init__(self):
        super().__init__('agent_env_node')

        # -------------------------- Data Fields ------------------------------------------------- #

        # -------------------------- Subscribe topics -------------------------------------------- #

        # -------------------------- Publish topics ---------------------------------------------- #
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.publisher_reset = self.create_publisher(
            Bool,
            'reset',
            10
        )

        # -------------------------- service ----------------------------------------------------- #
        self.env_data_client = self.create_client(VehicleEnvData, 'vehicle_env_data')

    def send_env_data_request(self):
        env_data_future = self.env_data_client.call_async(Empty())
        rclpy.spin_until_future_complete(self, env_data_future)
        return env_data_future.result()

    def observation_request(self) -> tuple[list[float], tuple[list[float], list[float], list[float]], int, bool]:
        return self.send_env_data_request()

    def action_request(self, linear: float, angle: float):
        twist_msg: Twist = Twist()

        vec3_linear: Vector3 = Vector3()
        vec3_linear.x = linear
        twist_msg.linear = vec3_linear

        vec3_angle: Vector3 = Vector3()
        vec3_angle.z = angle
        twist_msg.angular = vec3_angle

        self.publisher_cmd_vel.publish(twist_msg)

    def reset_env_request(self):
        reset_msg = Bool()
        reset_msg.data = True
        self.publisher_reset.publish(reset_msg)


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
