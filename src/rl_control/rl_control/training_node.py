import math
from math import isinf
import random
import rclpy
from .environment_node import Environment


from cares_reinforcement_learning.networks import TD3
from cares_reinforcement_learning.util import MemoryBuffer
import Actor
import Critic

import torch
import torch.nn as nn
import torch.optim as optim

if torch.cuda.is_available():
    DEVICE = torch.device('cuda')
    print("Working with GPU")
else:
    DEVICE = torch.device('cpu')
    print("Working with CPU")

BUFFER_CAPACITY = 1_000_000

GAMMA = 0.999
TAU = 0.001

ACTOR_LR = 1e-4
CRITIC_LR = 1e-3

STEPS = 150  # Hz
EPISODES = 10
BATCH_SIZE = 64

observation_size = 2
action_num = 2  # not 4

max_actions = [1.0, 1.0]
min_actions = [-1.0, -1.0]

env = Environment()


def main(args=None):
    rclpy.init(args=args)

    memory = MemoryBuffer(BUFFER_CAPACITY)

    actor = Actor(observation_size, action_num, ACTOR_LR, max_actions)
    critic_one = Critic(observation_size, action_num, CRITIC_LR)
    critic_two = Critic(observation_size, action_num, CRITIC_LR)

    td3 = TD3(
        actor_network=actor,
        critic_one=critic_one,
        critic_two=critic_two,
        max_actions=max_actions,
        min_actions=min_actions,
        gamma=GAMMA,
        tau=TAU,
        device=DEVICE
    )

    print(f"Filling Buffer...")     # buffer
    fill_buffer(memory)
    print(f"Buffer Filled!")

    print(f"Training Beginning")    # train
    train(td3, memory)

    env.destroy_node()              # end
    rclpy.shutdown()


def train(td3, memory: MemoryBuffer):   # TODO moving training steps into here

    # define a list to store all rewards
    historical_reward = []  # allocate space

    for ep in range(EPISODES):
        env.get_logger().info(f'{ep=} --------------------------------------------------------------')

        # TODO can we improve what we use for states, instead of pose, when it is relevant (is it relevant now?)
        # maybe involve localization
        [pose, _, _, _, _], _, _, _, _ = env.reset() # for now do we just care about the pose?
        episode_reward = 0

        for step in range(STEPS):   # TODO add correct action input to step() (NOTE: i don't know how to test this script, but it's a quick fix if you know how to)

            # select action using state
            with torch.no_grad():
                state_tensor = torch.FloatTensor(pose)  # convert to tensor so we can process it in pytorch
                state_tensor = state_tensor.unsqueeze(0)  # converts into a [1,x] tensor from a [x] tensor
                state_tensor = state_tensor.to(DEVICE)  # use GPU
                action = td3.forward(state_tensor)  # returns an action of the form ... todo confirm the shape
                action = action.cpu().data.numpy()

            # extract actions
            # todo find how the returned action from the forward pass is structured
            # this may not be correct, how is the return (x) structured?
            action_in = action[0]

            # implement action
            [next_pose, _, _, _, _], reward, terminated, _, _ = env.step(action)
            env.get_logger().info(f"{step=}, {reward=}")

            memory.add(pose, action, reward, next_pose, terminated)  # append into the memory array

            # learn
            experiences = memory.sample(BATCH_SIZE)
            for _ in range(0, 10):  # learn from 10 separate minibatch groups
                td3.learn(experiences)

            # update initial conditions for next step
            pose = next_pose
            episode_reward += reward    # running reward

            # exit episode for correct conditions
            if terminated:  # our step doesn't return truncated
                break

            # update historical rewards
            historical_reward.append(episode_reward)
            print(f"Episode #{ep} Reward {episode_reward}")


def fill_buffer(memory):    # TODO modify for the simulation env
    while len(memory.buffer) < memory.buffer.maxlen:

        [pose, _, _, _, _], _, _, _, _ = env.reset()   # reset episode throughout buffer fill

        while True:
            # get random float within action space
            action = (random.uniform(min_actions[0], max_actions[0]), random.uniform(min_actions[1], max_actions[1]))

            [next_pose, _, _, _, _], reward, terminated, _, _ = env.step(action)   # step with action

            memory.add(pose, action, reward, next_pose, terminated)

            pose = next_pose

            if terminated:  # todo set terminate to true when?
                break


if __name__ == '__main__':
    main()
