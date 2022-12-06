import rclpy
import time
from .agent_env_node import AgentEnvNode


def main(args=None):
    rclpy.init(args=args)
    agent_env_node = AgentEnvNode()
    time.sleep(0.5)
    agent_env_node.get_logger().info("------------- Starting the training -------------")

    agent_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
