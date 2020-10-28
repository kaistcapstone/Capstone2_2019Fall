import os
import numpy as np
import torch
import torch.nn as nn
import time

import simulatorfortest
import cv2

frame_history_len = 4

dtype = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor
env = simulatorfortest.Task(debug_flag=True, test_flag=False, state_blink=False, state_inaccurate=True)


if len(env.observation_space.shape) == 1:
    # This means we are running on low-dimensional observations (e.g. RAM)
    input_arg = env.observation_space.shape[0]
else:
    img_h, img_w, img_c = env.observation_space.shape
    input_arg = frame_history_len * img_c

test_model = torch.load('DQN_net1029_test (copy).pt')
print('DQN_net1029_test (copy).pt')

input_image = np.zeros((input_arg,img_w,img_h), np.uint8)


obs = env.reset()
for i in range(frame_history_len):
    input_image[i]=obs[0]

num_step=0
done=False

while(1):

    if frame_history_len >= num_step > 1:
        # input_image[0]=input_image[1]
        # input_image[1]=input_image[2]
        # input_image[2]=input_image[3]
        # input_image[3]=obs[:,:,0]
        input_image = input_image[frame_history_len-num_step:]
        input_image = input_image.tolist()
        obsinput = obs[:,:,0]
        obstolist = obsinput.tolist()
        for i in range(frame_history_len-num_step):
            input_image.append(obstolist)
        #input_image[-1] = obs[:,:,0]
        #np.append(input_image, [obs[:,:,0]],axis = 0)
        input_image = np.array(input_image)
    elif num_step > frame_history_len:
        input_image = input_image[1:]
        input_image = input_image.tolist()
        obsinput = obs[:,:,0]
        obstolist = obsinput.tolist()
        input_image.append(obstolist)
        #input_image[-1] = obs[:,:,0]
        #np.append(input_image, [obs[:,:,0]],axis = 0)
        input_image = np.array(input_image)

    elif num_step==1:
        for i in range(frame_history_len):
             input_image[i]=obs[:,:,0]
    # elif num_step==2:
    #     input_image[2]=obs[:,:,0]
    #     input_image[3]=obs[:,:,0]
    # elif num_step==3:
    #     input_image[1]=input_image[2]
    #     input_image[2]=obs[:,:,0]
    #     input_image[3]=obs[:,:,0]



    image = torch.from_numpy(input_image).type(dtype).unsqueeze(0)/255.0
    print(input_image.shape)
    print(image.shape)
    action =torch.IntTensor([[test_model(image).data.max(1)[1].cpu()]])[0,0]
#######################################################################################################
    if action == 0:
        action_word = "Front"
    if action == 1:
        action_word = "Left_turn"
    if action == 2:
        action_word = "Right_turn"
    if action == 3:
        action_word = "Back"
#######################################################################################################

    print(test_model(image).data, action_word, num_step)

    obs, reward, done = env.step(action)
    #time.sleep(0.03)

    num_step=num_step+1
    if done:
        print("===================================================")
        print("Steps taken: ",num_step,"Final score: ", env.score,"Balls_picked: ", len(env.balls_picked))
        print("===================================================")
        obs = env.reset()
        for i in range(4):
            input_image[i]=obs[:,:,0]
        num_step=0
        #time.sleep(0.5)
