
// 이 node는 공이 중복 카운트 될시 중복 카운트 된 데이터를 무효 처리한다.
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "core_msgs/ball_position_2.h"
#include "core_msgs/ball_position_modify.h"

#include "opencv2/opencv.hpp"
using namespace std;

float acceptable_distance = 0.05; //change this parameter to choose acceptable distance of deleting 중복된 ball data
ros::Publisher pub;

void imageCallback(const core_msgs::ball_position_2::ConstPtr& position1)
{
  int b_size_original = position1 -> b_size;
  int r_size_original = position1 -> r_size;
  int g_size_original = position1 -> g_size;
  cout << "original blue balls:  " << b_size_original << endl;
  cout << "original red balls:  " << r_size_original << endl;
  cout << "original green balls: "<< g_size_original << endl;
  int b_array = position1 -> b_img_x.size();
  int r_array = position1 -> r_img_x.size();
  int g_array = position1 -> g_img_x.size();
  cout << "array size blue balls:  " << b_array << endl;
  cout << "array size red balls:  " << r_array << endl;
  cout << "array size green balls: "<< g_array << endl;

  float b_pos_x_original[20];
  float b_pos_y_original[20];
  float b_pos_z_original[20];
  float r_pos_x_original[20];
  float r_pos_y_original[20];
  float r_pos_z_original[20];
  float g_pos_x_original[20];
  float g_pos_y_original[20];
  float g_pos_z_original[20];

  float b_radius_original[20];
  float r_radius_original[20];
  float g_radius_original[20];

  int b_data_reduction = 0, r_data_reduction = 0, g_data_reduction = 0; //number of reduction of data array

  cout << "checkpoint 1" <<endl;
  if (b_size_original != 0 ){
    for( size_t i = 0; i< b_size_original; i++ ){
      b_pos_x_original[i] = position1 -> b_img_x[i];
      b_pos_y_original[i] = position1 -> b_img_y[i];
      b_pos_z_original[i] = position1 -> b_img_z[i];
      b_radius_original[i] = position1 -> b_radius[i];
    }
  }
  if (r_size_original != 0){
    for( size_t i = 0; i< r_size_original; i++ ){
      r_pos_x_original[i] = position1 -> r_img_x[i];
      r_pos_y_original[i] = position1 -> r_img_y[i];
      r_pos_z_original[i] = position1 -> r_img_z[i];
      r_radius_original[i] = position1 -> r_radius[i];
    }
  }
  if (g_size_original != 0){
    for( size_t i = 0; i< g_size_original; i++ ){
      g_pos_x_original[i] = position1 -> g_img_x[i];
      g_pos_y_original[i] = position1 -> g_img_y[i];
      g_pos_z_original[i] = position1 -> g_img_z[i];
      g_radius_original[i] = position1 -> g_radius[i];
    }

  }


  cout << "checkpoint 2" <<endl;

//check blueball data
  if(b_size_original > 1){
    for( size_t i = 0; i< b_size_original; i++ ){
      for( size_t j = 0; j< b_size_original; j++ ){
        if(i < j){
          float x_distance = b_pos_x_original[i]-b_pos_x_original[j]; //get distance from two data point
          float y_distance = b_pos_y_original[i]-b_pos_y_original[j];
          float larger_radius;
          int noise_node;
          if(b_radius_original[i]>=b_radius_original[j]){
            larger_radius = b_radius_original[i];
            noise_node = j;
          }
          else{
            larger_radius = b_radius_original[j];
            noise_node = i;
          }
          // if x_distance and y_distance is small enough, eliminate one data
          if(pow(x_distance,2) + pow(y_distance,2) < pow(larger_radius,2)){
            if (noise_node == j){
                //put 100 in uneeded data
                b_pos_x_original[j] = 100+1000*j;
                b_pos_y_original[j] = 100+1000*j;
                b_pos_z_original[j] = 100+1000*j;
                ++b_data_reduction;
            }
            if (noise_node == i){
                b_pos_x_original[i] = 100+1000*i;
                b_pos_y_original[i] = 100+1000*i;
                b_pos_z_original[i] = 100+1000*i;
                ++b_data_reduction;
            }
          }
        }
      }
    }
  }


//check redball data
  if(r_size_original > 1){
    for( size_t i = 0; i< r_size_original; i++ ){
      for( size_t j = 0; j< r_size_original; j++ ){
        if(i < j){
          float x_distance = r_pos_x_original[i]-r_pos_x_original[j]; //get distance from two data point
          float y_distance = r_pos_y_original[i]-r_pos_y_original[j];
          float larger_radius;
          int noise_node;
          if(r_radius_original[i]>=r_radius_original[j]){
            larger_radius = r_radius_original[i];
            noise_node = j;
          }
          else{
            larger_radius = r_radius_original[j];
            noise_node = i;
          }
          // if x_distance and y_distance is small enough, eliminate one data
          if(pow(x_distance,2) + pow(y_distance,2) < pow(larger_radius,2)){
            if (noise_node == j){
                //put 100 in uneeded data
                r_pos_x_original[j] = 100+1000*j;
                r_pos_y_original[j] = 100+1000*j;
                r_pos_z_original[j] = 100+1000*j;
                ++r_data_reduction;
            }
            if (noise_node == i){
                r_pos_x_original[i] = 100+1000*i;
                r_pos_y_original[i] = 100+1000*i;
                r_pos_z_original[i] = 100+1000*i;
                ++r_data_reduction;
            }
          }
        }
      }
    }
  }

//check greenball data
if(g_size_original > 1){
  for( size_t i = 0; i< g_size_original; i++ ){
    for( size_t j = 0; j< g_size_original; j++ ){
      if(i < j){
        float x_distance = g_pos_x_original[i]-g_pos_x_original[j]; //get distance from two data point
        float y_distance = g_pos_y_original[i]-g_pos_y_original[j];
        float larger_radius;
        int noise_node;
        if(g_radius_original[i]>=g_radius_original[j]){
          larger_radius = g_radius_original[i];
          noise_node = j;
        }
        else{
          larger_radius = g_radius_original[j];
          noise_node = i;
        }
        // if x_distance and y_distance is small enough, eliminate one data
        if(pow(x_distance,2) + pow(y_distance,2) < pow(larger_radius,2)){
          if (noise_node == j){
              //put 100 in uneeded data
              g_pos_x_original[j] = 100+1000*j;
              g_pos_y_original[j] = 100+1000*j;
              g_pos_z_original[j] = 100+1000*j;
              ++g_data_reduction;
          }
          if (noise_node == i){
              g_pos_x_original[i] = 100+1000*i;
              g_pos_y_original[i] = 100+1000*i;
              g_pos_z_original[i] = 100+1000*i;
              ++g_data_reduction;
          }
        }
      }
    }
  }
}
  cout << "checkpoint 3" <<endl;

//define modified position
core_msgs::ball_position_2 mmsg;  //create a message for ball positions

  mmsg.b_img_x.resize(b_size_original-b_data_reduction);
  mmsg.b_img_y.resize(b_size_original-b_data_reduction);
  mmsg.b_img_z.resize(b_size_original-b_data_reduction);
  mmsg.r_img_x.resize(r_size_original-r_data_reduction);
  mmsg.r_img_y.resize(r_size_original-r_data_reduction);
  mmsg.r_img_z.resize(r_size_original-r_data_reduction);
  mmsg.g_img_x.resize(g_size_original-g_data_reduction);
  mmsg.g_img_y.resize(g_size_original-g_data_reduction);
  mmsg.g_img_z.resize(g_size_original-g_data_reduction);


  cout << "checkpoint 4" <<endl;

  mmsg.b_size = b_size_original - b_data_reduction;
  mmsg.r_size = r_size_original - r_data_reduction;
  mmsg.g_size = g_size_original - g_data_reduction;

  //int a = mmsg -> b_size_m;
  //int b = mmsg -> r_size_m;
  //int c = mmsg -> g_size_m;
  //checking message is well saved
  cout << "reduced blue balls:  " << b_data_reduction << endl;
  cout << "reduced red balls:  " << r_data_reduction << endl;
  cout << "reduced green balls: "<< g_data_reduction << endl;

  //save useful data in array
  int j = 0;
  for( size_t i = 0; i< b_size_original; i++ ){
    if(b_pos_x_original[i] < 99){
      mmsg.b_img_x[j] = b_pos_x_original[i];
      mmsg.b_img_y[j] = b_pos_y_original[i];
      mmsg.b_img_z[j] = b_pos_z_original[i];
      j++;

    }
  }
  j=0;
  for( size_t i = 0; i< r_size_original; i++ ){
    if(r_pos_x_original[i] < 99){
      mmsg.r_img_x[j] = r_pos_x_original[i];
      mmsg.r_img_y[j] = r_pos_y_original[i];
      mmsg.r_img_z[j] = r_pos_z_original[i];
      j++;
    }
  }
  j=0;
  for( size_t i = 0; i< g_size_original; i++ ){
    if(g_pos_x_original[i] < 99){
      mmsg.g_img_x[j] = g_pos_x_original[i];
      mmsg.g_img_y[j] = g_pos_y_original[i];
      mmsg.g_img_z[j] = g_pos_z_original[i];
      j++;
    }
  }

  pub.publish(mmsg);


}



int main(int argc, char **argv)
{
   ros::init(argc, argv, "modify_ball_count_node2"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   pub = nh.advertise<core_msgs::ball_position_2>("/position_modify2", 100); //setting publisher
   ros::Subscriber sub1 = nh.subscribe<core_msgs::ball_position_2>("/position2", 1000, imageCallback);

   ros::spin(); //spin.
   return 0;
}
