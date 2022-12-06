import rclpy

from .agent_env_node import AgentEnvNode


def main(args=None):
    rclpy.init(args=args)
    agent_env_node = AgentEnvNode()

    agent_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
