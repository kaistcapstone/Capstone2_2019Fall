#! /usr/bin/env python3
import os
import sys
import numpy as np
import torch
import torch.nn as nn
import roslib

from std_msgs.msg import Int8
from sensor_msgs.msg import CompressedImage

import cv2
import rospy, roslib, rospkg

input_number=4
img_rows=93
img_cols=93
count_image=0

dtype = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor



rospack = rospkg.RosPack()
root = rospack.get_path('dqn')
#path = root+"/src/nuelnetwork/learningrate=0.0002,gamma=0.99,~12balls,eps=0.2,nosee,noback,action3,region1_1130.pt"
path = root+"/src/nuelnetwork/learningrate=0.0002,gamma=0.99,~12balls,eps=0.2,nosee,action4,region1_1128.pt"

if torch.cuda.is_available():
    test_model = torch.load(path)
    print('.pt was loaded')
else:
    test_model=torch.load(path, map_location='cpu')
    print('.pt was loaded')


input_image = np.zeros((input_number,img_rows,img_cols), np.uint8)


class rl_dqn_network:
    def __init__(self):
        self.image_sub = rospy.Subscriber("RL_state/image",CompressedImage,self.callback, queue_size=1)

        # self.image_sub = rospy.Subscriber("RL_state/image",CompressedImage,self.callback, queue_size=1,buff_size=2**24)

    def callback(self,data):
        global count_image
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr ,cv2.IMREAD_GRAYSCALE )
        # print(data.data.size)
        if count_image==0:
            input_image[0]=cv_image
            input_image[1]=cv_image
            input_image[2]=cv_image
            input_image[3]=cv_image

        if count_image%2==0:
            cv2.waitKey(3)
            input_image[0]=input_image[1]
            input_image[1]=input_image[2]
            input_image[2]=input_image[3]
            input_image[3]=cv_image
        count_image+=1
        cv2.imshow("Image window", cv_image)

        image = torch.from_numpy(input_image).type(dtype).unsqueeze(0)/255.0
        action =torch.IntTensor([[test_model(image).data.max(1)[1].cpu()]])[0,0]
        print('publishing action is ',action)
        pub = rospy.Publisher('action/int8', Int8, queue_size=1)
        pub.publish(int(action.item()))


def main(args):
    rospy.init_node('rl_dqn_network', anonymous=False)

    rl = rl_dqn_network()
    # rospy.init_node('rl_dqn_network', anonymous=False)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
