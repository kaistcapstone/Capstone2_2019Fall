import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.autograd as autograd
import numpy as np

USE_CUDA = torch.cuda.is_available()
Variable = lambda *args, **kwargs: autograd.Variable(*args, **kwargs).cuda() if USE_CUDA else autograd.Variable(*args, **kwargs)

class DQN(nn.Module):
    #def __init__(self, in_channels=4, num_actions=10):
    def __init__(self, in_channels=8, num_actions=4):
        """
        Initialize a deep Q-learning network as described in
        https://storage.googleapis.com/deepmind-data/assets/papers/DeepMindNature14236Paper.pdf
        Arguments:
            in_channels: number of channel of input.
                i.e The number of most recent frames stacked together as describe in the paper
            num_actions: number of action-value to output, one-to-one correspondence to action in game.
        """
        super(DQN, self).__init__()
        self.conv1 = nn.Conv2d(in_channels, 32, kernel_size=8, stride=4)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=4, stride=2)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1)
        self.fc4 = nn.Linear(8 * 8 * 64, 512)
        self.fc5 = nn.Linear(512, num_actions)

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.fc4(x.view(x.size(0), -1)))
        return self.fc5(x)

class DuelingDQN(nn.Module):
    def __init__(self, in_channels=8, num_actions=4):
        """
        The basic structure of this code is based on
        https://github.com/higgsfield/RL-Adventure/blob/master/3.dueling%20dqn.ipynb
        Arguments:
            in_channels: number of channel of input.
                i.e The number of most recent frames stacked together as describe in the paper
            num_actions: number of action-value to output, one-to-one correspondence to action in game.
        """
        super(DuelingDQN, self).__init__()
        # self.conv1 = nn.Conv2d(in_channels, 32, kernel_size=8, stride=4)
        # self.conv2 = nn.Conv2d(32, 64, kernel_size=4, stride=2)
        # self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1)
        # self.fc4 = nn.Linear(8 * 8 * 64, 512)
        # self.fc5 = nn.Linear(512, num_actions)
        self.in_channels = in_channels
        self.num_actions = num_actions
        self.features = nn.Sequential(
            nn.Conv2d(in_channels, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU()
        )

        self.advantage = nn.Sequential(
            nn.Linear(8 * 8 * 64, 512),
            nn.ReLU(),
            nn.Linear(512, num_actions)
        )

        self.value = nn.Sequential(
            nn.Linear(8 * 8 * 64, 512),
            nn.ReLU(),
            nn.Linear(512, num_actions)
        )

    # def forward(self, x):
    #     x = F.relu(self.conv1(x))
    #     x = F.relu(self.conv2(x))
    #     x = F.relu(self.conv3(x))
    #     x = F.relu(self.fc4(x.view(x.size(0), -1)))
    #     return self.fc5(x)
    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1)
        advantage = self.advantage(x)
        value     = self.value(x)
        return value + advantage  - advantage.mean()

    # def feature_size(self):
    #     return self.features(autograd.Variable(torch.zeros(1, *self.in_channels))).view(1, -1).size(1)
    #
    # def act(self, state, epsilon):
    #     if random.random() > epsilon:
    #         state   = Variable(torch.FloatTensor(np.float32(state)).unsqueeze(0), volatile=True)
    #         q_value = self.forward(state)
    #         action  = q_value.max(1)[1].data[0]
    #     else:
    #         action = random.randrange(env.action_space.n)
    #     return action
