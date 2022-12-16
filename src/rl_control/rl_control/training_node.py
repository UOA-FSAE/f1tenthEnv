from math import isinf
import math
from random import random

import rclpy
import time
from .environment_node import Environment
from rclpy.task import Future

# from agents import TD3Agent
# from networks import Actor, Critic
# from util import MemoryBuffer


import math


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


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

            (pose, target, imu, lidar, odometry), reward, terminated, _, _ = env.step([random(), turn_angl])

            angle = euler_from_quaternion(
                odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y,
                odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w
            )

            accel = (odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z, \
                     odometry.twist.twist.angular.x, odometry.twist.twist.angular.y, odometry.twist.twist.angular.z)

            lidar = list(map(lambda x: x if not isinf(x) else -1, lidar.ranges.tolist()))

            env.get_logger().info(f"{step=}, {reward=}\n"
                                  f"{angle=}")

            if terminated:
                break

    env.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
