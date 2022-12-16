from math import isinf

import rclpy
import time
from .environment_node import Environment
from rclpy.task import Future

# from agents import TD3Agent
# from networks import Actor, Critic
# from util import MemoryBuffer


def main(args=None):
    rclpy.init(args=args)
    env = Environment()
    env.reset()
    env.get_logger().info("Hello World!")

    env.get_logger().info("Stepping Now!")
    while True:
        (pose, target, imu, lidar, odometry), reward, terminated, _, _ = env.step([1.0, 1.0])
        lidar = list(map(lambda x: x if not isinf(x) else -1, lidar.ranges.tolist()))
        env.get_logger().info(f"{reward=}")

        if terminated:
            env.reset()

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
