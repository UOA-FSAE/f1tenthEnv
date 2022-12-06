import rclpy
from rclpy.node import Node

import time
import random

from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Vector3, Twist

from service_interface.srv import VehicleEnvData


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
        self.env_data_client = self.create_client(VehicleEnvData, 'get_env_data')
        self.env_data_req = VehicleEnvData.Request()
        while not self.env_data_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_env_data_request(self) -> VehicleEnvData:
        env_data_future = self.env_data_client.call_async(self.env_data_req)
        rclpy.spin_until_future_complete(self, env_data_future)
        return env_data_future.result()

    @staticmethod
    def action_sample() -> tuple[float, float]:
        """
        This function returns a sample of the action space that can be used inside the environment.

        Returns:
            tuple[float, float] - a tuple contains two random floats for velocity and angle
        """
        return (random.random() * 2 - 1), (random.random() * 2 - 1)

    def observation_request(self) -> VehicleEnvData:
        """
        This functions returns an observation of the simulation.
        NOTE: you can not deconstruct this into a tuple as it returns the service object.


        Example:
            obs = AgentEnvNode().observation_request()

            lidar_data = obs.lidar

            imu_data = obs.imu

            reward = obs.reward

            termination = obs.termination

        Returns:
            service_interface.srv.VehicleEnvData

        """
        return self.send_env_data_request()

    def action_request(self, linear: float, angle: float):

        twist_msg: Twist = Twist()

        vec3_linear: Vector3 = Vector3()
        vec3_linear.x = linear
        vec3_linear.y = 0.0
        vec3_linear.z = 0.0
        twist_msg.linear = vec3_linear

        vec3_angle: Vector3 = Vector3()
        vec3_angle.x = 0.0
        vec3_angle.y = 0.0
        vec3_angle.z = angle
        twist_msg.angular = vec3_angle

        self.publisher_cmd_vel.publish(twist_msg)

    def reset_env_request(self):
        reset_msg = Bool()
        reset_msg.data = True
        self.publisher_reset.publish(reset_msg)
        time.sleep(1)


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
