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

    turn_angl = 0
    for ep in range(EPISODES):
        env.get_logger().info(f'{ep=} --------------------------------------------------------------')
        env.reset()

        for step in range(STEPS):
            if step % 10 == 0:
                turn_angl = random() * 4 - 2

            [pose, target, imu, lidar, odom], reward, terminated, _, _ = env.step((random(), turn_angl))

            env.get_logger().info(f"{step=}, {reward=}")

            if terminated:
                break

    env.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
