import torch.optim as optim

from dqn_model import DQN, DuelingDQN
#from dqn_learn import OptimizerSpec, dqn_learing
from DuelingDDQN_learnfortest import OptimizerSpec, dqn_learing
from utils.schedule import LinearSchedule
#! pip3 install opencv-python --user
#! pip3 install tensorboardX --user
#! pip3 install tensorflow --user
#! cd
#! tensorboard --logdir runs

# Please use below line in Ubuntu terminal to enable overcommit of memory
# echo 1 | sudo tee /proc/sys/vm/overcommit_memory

import simulatorfortest

BATCH_SIZE = 128 #32
GAMMA = 0.99
REPLAY_BUFFER_SIZE = 1000000
LEARNING_STARTS = 60000
LEARNING_FREQ = 12
FRAME_HISTORY_LEN = 8
TARGER_UPDATE_FREQ = 8000 #10000
LEARNING_RATE = 0.00055 #0.00025
ALPHA = 0.95
EPS = 0.01

def main(env):


    optimizer_spec = OptimizerSpec(
        constructor=optim.RMSprop,
        kwargs=dict(lr=LEARNING_RATE, alpha=ALPHA, eps=EPS),
    )

    exploration_schedule = LinearSchedule(1000000, 0.1)

    dqn_learing(
        env=env,
        q_func=DuelingDQN,
        optimizer_spec=optimizer_spec,
        exploration=exploration_schedule,
        replay_buffer_size=REPLAY_BUFFER_SIZE,
        batch_size=BATCH_SIZE,
        gamma=GAMMA,
        learning_starts=LEARNING_STARTS,
        learning_freq=LEARNING_FREQ,
        frame_history_len=FRAME_HISTORY_LEN,
        target_update_freq=TARGER_UPDATE_FREQ,
    )

if __name__ == '__main__':

    env = simulatorfortest.Task(debug_flag=False, test_flag=False, state_blink=True, state_inaccurate=True)
    main(env)
