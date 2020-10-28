#! /usr/bin/env python3
import os
import sys
import numpy as np
import torch
import torch.nn as nn
import roslib
import torchvision
from torchvision import datasets, models, transforms


from std_msgs.msg import String
from std_msgs.msg import Float32
from core_msgs.msg import markermsg_R

import cv2
import rospy, roslib, rospkg
from PIL import Image

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


data_T= transforms.Compose([
        transforms.Resize(256),
        transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

rospack = rospkg.RosPack()
root = rospack.get_path('cnn_for_jetson')
#path = root+"/src/nuelnetwork/CNN_dogcat0810.pt"
#path = root+"/src/nuelnetwork/CNN_dogcat0810_ResNet50_dict2.pt"
filenames = "CNN_dogcat0810_ResNet50_dict2.pt"
path = root+"/src/nuelnetwork/"+filenames



if torch.cuda.is_available():
    device=torch.device("cuda")
    test_model = models.resnet50(pretrained=False)
    num_ftrs = test_model.fc.in_features
    test_model.fc = nn.Linear(num_ftrs, 2)
#    test_model=models.resnet50(pretrained=False, num_classes=2)
    test_model.load_state_dict(torch.load(path))
    test_model.to(device)
    test_model.eval()
#    test_model=torch.load(path)
#    test_model.eval()
else:
    test_model=torch.load(path, map_location='cpu')
print(filenames+' was loaded')



class catdog_cnn_network:
    def __init__(self):
        self.image_sub = rospy.Subscriber("cropped_img_R",markermsg_R,self.callback)

    def callback(self,data):
        for [available, num] in [[data.image1_available_R,1] ,[data.image2_available_R,2]]:
            if available==True and num==1:
                np_arr= np.fromstring(data.cimage1_R.data, np.uint8)
                cv_image= cv2.imdecode(np_arr ,cv2.IMREAD_COLOR )
                cv2.imshow("first_image_R", cv_image)
                cv2.waitKey(50)
            elif available==True and num==2:
                np_arr= np.fromstring(data.cimage2_R.data, np.uint8)
                cv_image= cv2.imdecode(np_arr,cv2.IMREAD_COLOR )
                cv2.imshow("second_image_R", cv_image)
                cv2.waitKey(50)
            else:
                continue
            cv_image=Image.fromarray(cv_image)
            input_transform=data_T(cv_image)
            input_tensor=torch.zeros([1,3, 224, 224]).to(device)
            input_tensor[0]=input_transform
            outputs = test_model(input_tensor)
            predictions, preds = torch.max(outputs, 1)
            pub = rospy.Publisher('catdog_R/String', String, queue_size=1)
            pub2 = rospy.Publisher('catdog_R/Float', Float32, queue_size=1)
            for x in ['cat','dog']:
                if x ==['cat','dog'][preds]:
                    pub.publish(x)
                    pub2.publish(predictions)
                    if num==1:
                        print('first image is ' ,x)
                        #print(data.image2_center)
                    else:
                        print('second image is ' ,x)


def main(args):

    rospy.init_node('catdog_cnn_network_R', anonymous=False)
    cnn = catdog_cnn_network()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
