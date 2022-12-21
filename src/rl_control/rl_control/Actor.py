import torch
import torch.nn as nn
import torch.optim as optim


class Actor(nn.Module):
    def __init__(self, observation_size, num_actions, learning_rate, max_action):
        super(Actor, self).__init__()

        self.max_action = max_action

        self.hidden_size = [128, 64, 32]

        self.h_linear_1 = nn.Linear(in_features=observation_size, out_features=self.hidden_size[0])
        self.h_linear_2 = nn.Linear(in_features=self.hidden_size[0], out_features=self.hidden_size[1])
        self.h_linear_3 = nn.Linear(in_features=self.hidden_size[1], out_features=self.hidden_size[2])
        self.h_linear_4 = nn.Linear(in_features=self.hidden_size[2], out_features=num_actions)

        self.optimiser = optim.Adam(self.parameters(), lr=learning_rate)

    def forward(self, state):
        x = torch.relu(self.h_linear_1(state))
        x = torch.relu(self.h_linear_2(x))
        x = torch.relu(self.h_linear_3(x))
        x = torch.tanh(self.h_linear_4(x)) * self.max_action[0]
        return x


class PolicyNetworkSAC(nn.Module):
    def __init__(self, vector_size, num_actions, learning_rate):
        super(PolicyNetworkSAC, self).__init__()

        self.log_std_min = -20
        self.log_std_max = 2

        self.num_actions = num_actions
        self.input_size  = vector_size
        self.hidden_size = [128, 64, 32]

        self.linear1 = nn.Linear(self.input_size,     self.hidden_size[0])
        self.linear2 = nn.Linear(self.hidden_size[0], self.hidden_size[1])

        self.mean_linear    = nn.Linear(self.hidden_size[1], self.num_actions)
        self.log_std_linear = nn.Linear(self.hidden_size[1], self.num_actions)

        self.optimiser = optim.Adam(self.parameters(), lr=learning_rate)

    def forward(self, state):
        x = torch.relu(self.linear1(state))
        x = torch.relu(self.linear2(x))

        mean    = self.mean_linear(x)
        log_std = self.log_std_linear(x)
        log_std = torch.clamp(log_std, self.log_std_min, self.log_std_max)

        return mean, log_std
