
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
#include <algorithm>
#include <math.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/pick.h"
#include "std_msgs/Int32.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"


#include "opencv2/opencv.hpp"


#define RAD2DEG(x) ((x)*180./M_PI)
#define OFFSET 360
#define FRQUENT 5
#define PORT 3000
#define IPADDR "172.16.0.1" // myRIO ip
#define RANGE 360
//range 안으로 들어오면 픽업
// #define IPADDR "127.0.0.1" // host computer ip

using namespace std;
float prev_ball_Y_R = 1000;
int pick_motion;
float prev_ball_Y  = 1000 ;
int JUDG_CON = 0 ;
int JUDG_CON_R = 0 ;
int now_pick;
int now_pick_r;
int past_now_pick = 0;
int past_now_pick_r = 0;
int ball_number;
int ball_number_R;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];
float ball_X_R[20];
float ball_Y_R[20];
float ball_distance_R[20];
float near_ball_y, near_ball_x;
float near_ball_y_R, near_ball_x_R;
int JUDE = 0;
int c_socket, s_socket;
struct sockaddr_in c_addr;
int curr_blue_ball=0;
int picked_ball_num=0;
float data[1];
//data[2]
int past_pick_w = 0;
float past_near_ball = 1000.000;


int picked_ball_num_R=0;
int past_pick_w_R = 0;
float past_near_ball_R = 1000.000;


#define RAD2DEG(x) ((x)*180./M_PI)

#define OFFSET2 400

void dataInit(){
  data[0]=0;

}

void ball_Callback(const std_msgs::Int32::ConstPtr& msg1){
    pick_motion = msg1->data;

}

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{

    int count = position->size;
		int count2 = position->size2;
    ball_number = count;
		ball_number_R = count2;

    for(int i = 0; i < count; i++)
    {
        ball_X[i] = position->img_x[i];
        ball_Y[i] = position->img_y[i];
		ball_distance[i] = ball_Y[i];
    }
		for(int i = 0; i < count2; i++)
    {
				ball_X_R[i] = position->img_x2[i];
        ball_Y_R[i] = position->img_y2[i];
		ball_distance_R[i] = ball_Y_R[i];
    }
}

void ball_distanceInit()
{
  for(int i = 0 ; i < 20 ;i++){
  ball_distance[i]=0;
  ball_distance_R[i]=0;
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sorting_node");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
    ros::Subscriber sub2 = nh.subscribe("JUDE", 1000, ball_Callback);


    dataInit();

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

		float near_ball, near_ball_R;


    int judge_constant = 0;
    int pick_w = 0;
    int judge_constant_R = 0;
    int pick_w_R = 0;


    while(ros::ok()){
			near_ball = 5;
			near_ball_R = 5;


      ros::spinOnce();


// -------------------sorting------------------------------
      ball_distanceInit();
      for(int i = 0; i < ball_number; i++)
			{
				if (ball_distance[i] < near_ball){
					near_ball = ball_distance[i];
				}
			}
			for(int i = 0; i < ball_number_R; i++)
			{
				if (ball_distance_R[i] < near_ball_R){
          near_ball_R = ball_distance_R[i];
        }
			}

      // case
      if(near_ball == 0 && near_ball_R == 0){
        data[0] = 0;
      }
			else{
        if(near_ball_R < near_ball){
          data[0] = 1;
        } // blue ball right
        else{
          data[0] = 0;
        } // red ball left
      }

        if(ball_number_R==0){past_near_ball_R=1000;}
        if(pick_motion==1&&(near_ball_R < near_ball)){
          int k = 0;
          while(k < 15){
            data[0]=1;
            write(c_socket, data, 4);
          ros::Duration(0.1).sleep();
          k++;
          }

        }


  write(c_socket, data, 4);
  printf("data is = %f\n", data[0]);

  ros::Duration(0.025).sleep();
}


  return 0;
}
