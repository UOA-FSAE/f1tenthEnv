import rclpy
import time
from src.rl_control.rl_control.environment_node import Environment
# from agents import TD3Agent
# from networks import Actor, Critic
# from util import MemoryBuffer


def main(args=None):
    rclpy.init(args=args)
    env = Environment()
    env.get_logger().info("Hello World!")

    env.get_logger().info("Resetting Simulation...")
    env.reset()
    env.get_logger().info("Stepping Now!")
    state, reward, terminated, _, _ = env.step([1.0, 1.0])
    time.sleep(5)
    env.get_logger().info("Resetting Simulation...")
    env.reset()

    # env.get_logger().info("------------- Starting the training -------------")

    # while True:
    #     env.reset_env_request()
    #     env.get_logger().info("------------- Reset Requested --------------")
    #
    #     env.action_request(1.0, 0.0)
    #     env.get_logger().info("------------- Action Requested -------------")
    #     time.sleep(5)


    # env.get_logger().info(f"{}")

    env.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
