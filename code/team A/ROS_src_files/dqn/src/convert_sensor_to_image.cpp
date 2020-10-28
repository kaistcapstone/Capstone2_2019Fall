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

cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC1);

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int ball_number;
float ball_X[20];
float ball_Y[20];


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

        ball_X[i] = position->img_x[i];
        ball_Y[i] = position->img_y[i];
        std::cout << "ball_X : "<< ball_X[i];
        std::cout << "   ball_Y : "<< ball_Y[i]<<std::endl;
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
    ros::Publisher pub;
    pub = n.advertise<sensor_msgs::CompressedImage>("RL_state/image", 1000); //setting publisher
    sensor_msgs::CompressedImage msg;


    while(ros::ok){
      cv::Mat map = cv::Mat::zeros(MAP_WIDTH*scale_up_size, MAP_HEIGHT*scale_up_size, CV_8UC1); //91*91 size image

      // Drawing Lidar data
      float obstacle_x, obstacle_y;
      int cx, cy;
      int cx1, cx2, cy1, cy2;
      for(int i = 0; i < lidar_size; i++)
      {
          obstacle_y = -lidar_distance[i]*sin(lidar_degree[i]);
          obstacle_x = -lidar_distance[i]*cos(lidar_degree[i]);

          cx = MAP_WIDTH/2 + (int)(obstacle_y/MAP_RESOL);
          cy = MAP_HEIGHT + (int)(obstacle_x/MAP_RESOL);

          if(check_point_range(cx,cy)){
              cv::rectangle(map,cv::Point(cx*scale_up_size, cy*scale_up_size),cv::Point(cx*scale_up_size+2, cy*scale_up_size+2), cv::Scalar(wall_color), -1);
          }
      }
      // Drawing ball
      for(int i = 0; i < ball_number; i++)
      {
          cx = MAP_WIDTH/2 + (int)(ball_X[i]/MAP_RESOL);
          cy = MAP_HEIGHT - (int)(ball_Y[i]/MAP_RESOL);

          if(check_point_range(cx,cy)){
            cv::rectangle(map,cv::Point(cx*scale_up_size, cy*scale_up_size),cv::Point(cx*scale_up_size+2, cy*scale_up_size+2), cv::Scalar(ball_color), -1);
          }
      }
      // Drawing ROBOT
      // cv::circle(map,cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),3,cv::Scalar(255,0,0),-1);
      cv::rectangle(map,cv::Point(((MAP_WIDTH/2)-2)*scale_up_size-1, (MAP_HEIGHT-2)*scale_up_size+1),cv::Point(((MAP_WIDTH/2)+3)*scale_up_size, MAP_HEIGHT*scale_up_size-1), cv::Scalar(robot_padding_color), -1);
      cv::rectangle(map,cv::Point(((MAP_WIDTH/2))*scale_up_size-1, (MAP_HEIGHT-1)*scale_up_size+1),cv::Point(((MAP_WIDTH/2)+1)*scale_up_size, MAP_HEIGHT*scale_up_size-1), cv::Scalar(robot_color), -1);

      // if it is posible to use cvbrideg, you can use below
      // msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", map).toImageMsg();

      // because we cannot use cv_bridge in pyton3
      // core_msgs::multiarray msg;
      // int rows = map.rows;
      // int cols = map.cols;
      // msg.data.resize(rows*cols);  //adjust the size of array
      // msg.cols=cols;
      // msg.rows=rows;
      // for(int i = 0; i < rows; i++){
      //   for(int j = 0; j < cols; j++){
      //     msg.data[i*rows+j]=map.at<uchar>(i,j);
      //   }
      // }



      cv::imencode(".jpg", map, msg.data);

      pub.publish(msg);

      cv::imshow("Frame",map);
      if(cv::waitKey(50)==113){  //wait for a key command. if 'q' is pressed, then program will be terminated.
        return 0;
      }
      ros::spinOnce();
    }

    return 0;
}
