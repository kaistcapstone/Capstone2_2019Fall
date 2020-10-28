#include <stdio.h>
#include <stdlib.h>
#include <cmath>                                                                //For calculation something
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>                                                          //For TCP/IP connection
#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>                                                 //For msg of the topic "/cmd_vel"
#include <std_msgs/Int32.h>

//For motor keyboard control
#include "kbhit.h"
#include "getch.h"
#include <iostream>
#include <queue>

using namespace std;


static int ball_pass[10] = {0,0,0,0,0,0,0,0,0,0};
static int prev_ball_pass[10] = {0,0,0,0,0,0,0,0,0,0};
static int i = 0;
static int ball_count = 0;
std_msgs::Int32 pmsg;
ros::Publisher pub;
void msgCallback(const std_msgs::Int32::ConstPtr& msg)                      //Receive topic /cmd_vel and write data[24] to send via TCP/IP
{
  cout << "callback start";
  ball_pass[i%10] = msg->data;
  int count = 0;

  for(int j = 0; j < 10; j++ ){
    if (prev_ball_pass[j] == 1) {
      count = 0;
      break;
    }
    else count = 1;
  }

  if(msg->data == 1){
    ball_count += count;
  }

  for(int j = 0; j < 10; j++ ){
    prev_ball_pass[j] = ball_pass[j];
  }

  i++;

  ROS_INFO("ball collected : %d",ball_count);

  pmsg.data = ball_count;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_count_node");
    ros::NodeHandle nh;
    ros::Subscriber twist_sub_ = nh.subscribe("/ball_check", 10, msgCallback);                //Subscriber for the topic "/cmd_vel", "/action/int8" to operate the motor
    pub = nh.advertise<std_msgs::Int32>("/picked_balls",100);
    while(ros::ok()){                                               //Send the data via TCP/IP
        // cout <<"while is running";
        ros::spinOnce();
        pub.publish(pmsg);
        ros::Duration(0.05).sleep();
      }


    return 0;
}
