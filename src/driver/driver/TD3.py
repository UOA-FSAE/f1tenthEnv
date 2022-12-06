import rclpy
import time
from .agent_env_node import AgentEnvNode


def main(args=None):
    rclpy.init(args=args)
    agent_env_node = AgentEnvNode()
    time.sleep(0.5)
    agent_env_node.get_logger().info("------------- Starting the training -------------")

    while True:
        agent_env_node.reset_env_request()
        agent_env_node.get_logger().info("------------- Reset Requested --------------")

        agent_env_node.action_request(1.0, 0.0)
        agent_env_node.get_logger().info("------------- Action Requested -------------")
        time.sleep(5)

    agent_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
