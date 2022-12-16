from math import isinf
from random import random

import rclpy
import time
from .environment_node import Environment
from rclpy.task import Future

# from agents import TD3Agent
# from networks import Actor, Critic
# from util import MemoryBuffer

STEPS = 150  # Hz
EPISODES = 10


def main(args=None):
    rclpy.init(args=args)
    env = Environment()
    env.reset()

    for ep in range(EPISODES):
        env.get_logger().info(f'{ep=}--------------------------------------------------------------')
        env.reset()

        turn_angl = random() * 2 - 1
        for step in range(STEPS):
            (pose, target, imu, lidar, odometry), reward, terminated, _, _ = env.step([random(), turn_angl])

            velocity = (odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z, \
                        odometry.twist.twist.angular.x, odometry.twist.twist.angular.y, odometry.twist.twist.angular.z)
            lidar = list(map(lambda x: x if not isinf(x) else -1, lidar.ranges.tolist()))

            env.get_logger().info(f"{step=}, {reward=}")

            if terminated:
                break

    env.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
