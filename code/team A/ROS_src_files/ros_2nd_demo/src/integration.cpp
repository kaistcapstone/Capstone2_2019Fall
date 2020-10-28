#include "ros/ros.h"
#include "ros_2nd_demo/Message1.h"

#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>

#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
// #define IPADDR "127.0.0.1" // host computer ip
#define IPADDR "172.16.0.1" // myRIO ip

float time_1=0.000;
float data[24];

int c_socket, s_socket;
struct sockaddr_in c_addr;

void dataInit()
{
	data[0] = 0; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	data[6] = 0; //GamepadStickAngle(_dev, STICK_RIGHT);
	data[7] = 0; //GamepadStickLength(_dev, STICK_RIGHT);
	data[8] = 0; //GamepadTriggerLength(_dev, TRIGGER_LEFT);
	data[9] = 0; //GamepadTriggerLength(_dev, TRIGGER_RIGHT);
	data[10] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_UP);
	data[11] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
	data[12] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
	data[13] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
	data[14] = 0; //GamepadButtonDown(_dev, BUTTON_A); // duct on/off
	data[15] = 0; //GamepadButtonDown(_dev, BUTTON_B);
	data[16] = 0; //GamepadButtonDown(_dev, BUTTON_X);
	data[17] = 0; //GamepadButtonDown(_dev, BUTTON_Y);
	data[18] = 0; //GamepadButtonDown(_dev, BUTTON_BACK);
	data[19] = 0; //GamepadButtonDown(_dev, BUTTON_START);
	data[20] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);
	data[21] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);
	data[22] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
	data[23] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);
}

void msgCallback(const ros_2nd_demo::Message1::ConstPtr& msg)
{
  printf("recieve msg = %.3f\n", msg->data_1);
  time_1 += 0.001;
  printf("current time %.3f\n", time_1);
  if (time_1<=3){
  data[5]=1; //직진
  data[9]=0;
  }
  else{
      if (time_1>3 && time_1<=5.06){
         data[9]=1;//오른 회전
         data[5]=0;
      }
      else{
          if(time_1>5.06 && time_1<8.06){
          data[5]=1;//go straight
          data[9]=0;
          }
          else{
          data[5]=0;//stop
          data[9]=0;
          }
      }

  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "integration");
  ros::NodeHandle nh;
  ros::Subscriber ros_2nd_sub = nh.subscribe("time", 100, msgCallback);

  c_socket = socket(PF_INET, SOCK_STREAM, 0);
  c_addr.sin_addr.s_addr = inet_addr(IPADDR);
  c_addr.sin_family = AF_INET;
  c_addr.sin_port = htons(PORT);

  ///////////////////////////////////////////////////////////////////////
  //	렙뷰와 통신이 되었는지 확인하는 코드 아래 코드를 활성화 후 노드를 실행 시켰을때///
  //	노드가 작동 -> 통신이 연결됨, Failed to connect 이라고 뜸 -> 통신이 안됨///
  ////////////////////////////////////////////////////////////////////////
  if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
      printf("Failed to connect\n");
      close(c_socket);
      return -1;
  }

  dataInit();

  // ros::spin();
  while(ros::ok){
    ros::spinOnce();
    write(c_socket, data, sizeof(data));
    ros::Duration(0.001).sleep();
  }

return 0;
}
