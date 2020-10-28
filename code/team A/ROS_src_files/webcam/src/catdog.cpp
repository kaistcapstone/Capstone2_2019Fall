#include <ros/ros.h>
#include "std_msgs/String.h"
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
// #include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

using namespace std;

#define PI 3.141592
#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
#define IPADDR "172.16.0.1" // host computer ip
// #define IPADDR "172.22.11.2" // myRIO ip
struct sockaddr_in c_addr;
int c_socket, s_socket;


// int cat_door = 0;
// int dog_door = 0;

float data[24];

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

void callback(const std_msgs::String::ConstPtr& msg){
  std::string catdog = msg->data;
  printf("The result is: %s\n", catdog.c_str());
  if(strcmp(catdog.c_str(), "cat")){
    data[14] = 0;
    data[15] = 1;
    printf("sending cat\n");
    // cat_door = 1;
    // dog_door = 0;
  }
  else if(strcmp(catdog.c_str(), "dog")){
    data[14] = 1;
    data[15] = 0;
    printf("sending dog\n");
    // cat_door = 0;
    // dog_door = 1;
  }
  else{
    dataInit();
    // cat_door = 0;
    // dog_door = 0;
    printf("sending nothing\n");
  }

}
int main(int argc, char **argv){
  ros::init(argc, argv, "catdog_result");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<std_msgs::String>("/catdog/String", 1000, callback);

  // ---------- tcp connection -------------
  c_socket = socket(PF_INET, SOCK_STREAM, 0);
  c_addr.sin_addr.s_addr = inet_addr(IPADDR);
  c_addr.sin_family = AF_INET;
  c_addr.sin_port = htons(PORT);
  if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
    printf("Failed to connect\n");
		close(c_socket);
		return -1;
  }

  while(ros::ok()){

    ros::spinOnce();
    write(c_socket, data, sizeof(data));
  }
  close(c_socket);
  return 0;
}
