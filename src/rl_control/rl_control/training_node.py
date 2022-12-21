import math
from math import isinf
from random import random

import rclpy
from .environment_node import Environment


STEPS = 150  # Hz
EPISODES = 10


def main(args=None):
    rclpy.init(args=args)
    env = Environment()
    env.reset()

    for ep in range(EPISODES):
        env.get_logger().info(f'{ep=} --------------------------------------------------------------')
        env.reset()

        for step in range(STEPS):

            [pose, target, imu, lidar, odom], reward, terminated, _, _ = env.step((1., 1.))

            env.get_logger().info(f"{step=}, {reward=}")

            if terminated:
                break

    env.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
