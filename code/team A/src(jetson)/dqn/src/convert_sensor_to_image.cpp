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
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include "core_msgs/ball_position.h"
#include "core_msgs/multiarray.h"


#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CompressedImage.h"

#include "opencv2/opencv.hpp"

using namespace std;

float MAP_RESOL=0.1;

int MAP_WIDTH = 31;
int MAP_HEIGHT = 31;
int scale_up_size=3;

int ball_color=255;
int wall_color=100;
int robot_color=200;
int robot_padding_color=150;

#define RAD2DEG(x) ((x)*180./M_PI)



boost::mutex map_mutex;
ros::Publisher pub;

cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC1);

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];
int near_ball;


void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    map_mutex.lock();
    int count = scan->scan_time / scan->time_increment;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
        lidar_distance[i]=scan->ranges[i];
    }
    map_mutex.unlock();
}
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
    map_mutex.lock();
    int count = position->size;
    ball_number=count;
    for(int i = 0; i < count; i++)
    {

        ball_X[i] = position->img_x[i] + 0.16; //0.16
        ball_Y[i] = position->img_y[i];
    }
    map_mutex.unlock();

}

bool check_point_range(int cx, int cy)
{
    return (cx<MAP_WIDTH)&&(cx>0)&&(cy<MAP_HEIGHT)&&(cy>0);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "covert_sensor_to_image");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position123", 1000, camera_Callback);
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<sensor_msgs::CompressedImage>("RL_state/image", 1); //setting publisher



    while(ros::ok()){
      cv::Mat map = cv::Mat::zeros(MAP_WIDTH*scale_up_size, MAP_HEIGHT*scale_up_size, CV_8UC1); //91*91 size image
      // Drawing Lidar data
      float obstacle_x, obstacle_y;
      int cx, cy;
      int cx1, cx2, cy1, cy2;


      // Drawing ball
      for(int i = 0; i < ball_number; i++)
      {
          cx = MAP_WIDTH/2 + (int)((ball_X[i]-0.13)/MAP_RESOL);
          cy = MAP_HEIGHT - (int)(ball_Y[i]/MAP_RESOL) + 1;
          if(ball_X[i]-0.16<-0.06 && ball_X[i]-0.16>-0.14){
          	cx = MAP_WIDTH/2;
	      }
          else if(ball_X[i]-0.16<=-0.14 && ball_X[i]-0.16>-0.18){ //-0.2
          	cx = MAP_WIDTH/2-1;
	      }
	      else if(ball_X[i]-0.16>=-0.06 && ball_X[i]-0.16<0.06){
	        cx = MAP_WIDTH/2+1;
          }
          else if(ball_X[i]-0.16<=-0.18 && ball_X[i]-0.16>-0.3){
             cx = MAP_WIDTH/2-2;
          }
          if(check_point_range(cx,cy)){
            cv::rectangle(map,cv::Point(cx*scale_up_size, cy*scale_up_size),cv::Point(cx*scale_up_size+2, cy*scale_up_size+2), cv::Scalar(ball_color), -1);
          }
      }

      for(int i = 0; i < lidar_size; i++)
      {
          map_mutex.lock();
          obstacle_y = -lidar_distance[i]*sin(lidar_degree[i])*1.5;
          obstacle_x = -lidar_distance[i]*cos(lidar_degree[i])*1.5;
          map_mutex.unlock();

          cx = MAP_WIDTH/2 + (int)(obstacle_y/MAP_RESOL);
          cy = MAP_HEIGHT + (int)(obstacle_x/MAP_RESOL) + 3;

          if(check_point_range(cx,cy)){
              cv::rectangle(map,cv::Point(cx*scale_up_size, cy*scale_up_size),cv::Point(cx*scale_up_size+2, cy*scale_up_size+2), cv::Scalar(wall_color), -1);
          }
      }

      // Drawing high reward region
      cv::rectangle(map,cv::Point((14)*scale_up_size-1,(30)*scale_up_size+1),cv::Point((16)*scale_up_size,(31)*scale_up_size), cv::Scalar(ball_color), -1);
      // Drawing low reward region
      cv::rectangle(map,cv::Point((14)*scale_up_size-1,(28)*scale_up_size+1),cv::Point((16)*scale_up_size,(30)*scale_up_size), cv::Scalar(robot_padding_color), -1);
      // Drawing penalty region 1(one on the right)
      cv::rectangle(map,cv::Point((16)*scale_up_size,(28)*scale_up_size+1),cv::Point((19)*scale_up_size-1,(31)*scale_up_size), cv::Scalar(robot_color), -1);
      // Drawing penalty region 2(one on the left)
      cv::rectangle(map,cv::Point((11)*scale_up_size,(28)*scale_up_size+1),cv::Point((14)*scale_up_size-1,(31)*scale_up_size), cv::Scalar(robot_color), -1);

      sensor_msgs::CompressedImage msg;
      cv::imencode(".jpg", map, msg.data);

      pub.publish(msg);

      cv::imshow("Frame",map);
      cv::waitKey(30);

      ros::spinOnce();
    }

    return 0;
}
