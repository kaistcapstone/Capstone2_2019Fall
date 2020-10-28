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
#include <geometry_msgs/Twist.h>                                                 //For msg of the topic "/cmd_vel"

#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/mark.h"
#include "core_msgs/goal_sw.h"

#include "core_msgs/ball_bottom.h"
#include "core_msgs/ball_position.h"


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;


core_msgs::goal_sw g_msg;
#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
#define IPADDR "172.16.0.1"// myRIO ipadress
int lidar_size; float lidar_degree[400]; float lidar_distance[400];
float lidar_obs; float k2=3.0; // constant
float k3=26; float c1 = 40;
float c2 = 15; float c3 = 710; //int ball_number; //float ball_X[20]; //float ball_Y[20];
//float ball_distance[20]; //int near_ball;
int count_num=0;
int action;
int	sw = 3;
int ssw =0;
int sw2 = 0;
int mark_sw=0;
int c=0;
int x=0;
int z=0;
int x_1;
int x_2;
int y_1;
int y_2;
int z_1;
int z_2;
int after_slam_goforward=0;
int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
int straight;
int t3;
int countball=0;
int rotat90=0;
int secondrelease;
float data[24];
int eatball=0;
int count2=0;
float cmdvelx=0;
float cmdvely=0;
float posex;
float posey;
float posez;
int catordog;
int nomarkersee;
int trans1=0;
int trans2=0;
#define RAD2DEG(x) ((x)*180./M_PI)
void take_sleep(int t){
	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	for (int i=0; i<t; i++){
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
}
void turn_left(float size, int t){
	data[0] = size; //lx*data[3];
	data[1] = size; //ly*data[3];
	data[2] = size; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = size; //GamepadStickLength(_dev, STICK_LEFT);
	for (int i=0; i<t; i++){
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
}
void turn_right(float size, int t){
	data[0] = -size; //lx*data[3];
	data[1] = -size; //ly*data[3];
	data[2] = -size; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = -size; //GamepadStickLength(_dev, STICK_LEFT);
	for (int i=0; i<t; i++){
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
}
void go_front(float size, int t){
	data[0] = -size; //lx*data[3];
	data[1] = +size; //ly*data[3];
	data[2] = -size; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = +size; //GamepadStickLength(_dev, STICK_LEFT);
	for (int i=0; i<t; i++){
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
}

void go_back(float size, int t){
	data[0] = +size; //lx*data[3];
	data[1] = -size; //ly*data[3];
	data[2] = +size; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = -size; //GamepadStickLength(_dev, STICK_LEFT);
	for (int i=0; i<t; i++){
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
}
void go_left(float size, int t){
	data[0] = +size; //lx*data[3];
	data[1] = +size; //ly*data[3];
	data[2] = -size; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = -size; //GamepadStickLength(_dev, STICK_LEFT);
	for (int i=0; i<t; i++){
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
}
void go_right(float size, int t){
	data[0] = -size; //lx*data[3];
	data[1] = -size; //ly*data[3];
	data[2] = +size; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = +size; //GamepadStickLength(_dev, STICK_LEFT);
	for (int i=0; i<t; i++){
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
}
void go_del(float del_x, float del_y, float rot, int t){
    data[0] = -k3*(del_x+del_y+rot*0.35);
    data[1] = +k3*(del_x-del_y-rot*0.35);
    data[2] = -k3*(del_x-del_y+rot*0.35);
    data[3] = +k3*(del_x+del_y-rot*0.35);
		for (int i=0; i<t; i++){
			write(c_socket, data, sizeof(data));
			ros::Duration(0.025).sleep();
		}
}
int closest_ball( vector<float> ball_array){
	int result_idx = -1;
	if (ball_array.size() ==0) return -1;
	float front_dist = ball_array[0];

	for (int i=0; i < ball_array.size(); i++)
	{
		if (ball_array[i] <=front_dist){
			front_dist = ball_array[i];
			result_idx = i;
		}
	}
	return result_idx;
}
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
void rgbd_ball_Callback(const core_msgs::ball_position::ConstPtr& position){
	count_num = position->size;
}
void webcam_Callback(const core_msgs::ball_bottom::ConstPtr& position){
	count2 = position->size;
	count_num +=count2;
	cout << count_num<<"ball num"<<endl;
	int closest_idx = closest_ball(position-> img_z);
	if ( position ->size != 0){
	 x =position->img_x[closest_idx];
	 z =position->img_z[closest_idx];
	 c = position->color[closest_idx];
	}
}
void rl_action_Callback(const std_msgs::Int8::ConstPtr& msg){
	action = msg->data;
}
void PoseUpdate(const geometry_msgs::PoseStampedConstPtr& pose){
	geometry_msgs::PoseStampedConstPtr pose_ptr_;
	pose_ptr_ = pose;
	posex = 	pose_ptr_->pose.position.x;
	posey =	pose_ptr_->pose.position.y;
	posez =   pose_ptr_->pose.orientation.z;
}
void msgCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	cout<<"cmd_vel"<<endl;
	cmdvelx=cmd_vel->linear.x;
	cmdvely=cmd_vel->linear.y;
}                    //Receive topic /cmd_vel and write data[24] to send via TCP/IP

void marker_Callback(const core_msgs::mark::ConstPtr& msg){
	x_1 = msg->x1;
	y_1 = msg->y1;
	z_1 = msg->z1;
	x_2 = msg->x2;
	y_2 = msg->y2;
	z_2 = msg->z2;
	// cout << "mark"<<x_1 <<y_1 <<z_1 <<endl;
	mark_sw=1;
}
void catdog_cnn_Callback(const std_msgs::String::ConstPtr& msg){
	std::string catdog = msg->data;
	if (catdog=="cat"){
		catordog=1;
	}
	else if (catdog=="dog"){
		catordog=2;
	}
}
void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){
	int count = scan->scan_time / scan->time_increment;
	lidar_size=count;
	for(int i = 0; i < count; i++)
	{
			lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
			lidar_distance[i]=scan->ranges[i];
	}
}
void align(float z, float vel){
	float ang=0;
	int t=0;
	dataInit();
	cout << "align"<<endl;
	if (z>0){
		ang = 180 - 180*z;
		t = ang * 17.5 / 15 ;
		turn_left(vel,t);
	}
	else {
		ang = 180 +180*z;
		t = ang * 17.5 / 15 ;
		turn_right(vel,t);
	}
	dataInit();
}
void move_trans (float y, float goal, float vel){
	float t=0;
	cout <<"move_trans" << endl;
	if ( y < goal){
		//go_right
		t= int((goal-y) * 100/0.63);
		go_del(0,1 /26 *vel ,0 ,t);
	}
	else {
		 //go_ left
		t= int((y-goal) * 100/0.76);
		go_del(0,-1 /26 *vel,0,t);
	}

	dataInit();
}
void go_forward(float x, float goal, float vel){
	int t=0;
	cout << "go_forward" << endl;
	if ( x > goal){
		t = (x-goal) * 50 / 0.465;
		if (vel == 50) {
			t = (x-goal) * 100 / 1.435;
		}
		go_front(vel,t);
		dataInit();
	}
}
int main(int argc, char **argv)
{


    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;
		mark_sw=0;
	ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1, rgbd_ball_Callback);
  ros::Subscriber sub2 = n.subscribe<core_msgs::ball_bottom>("/position2", 1, webcam_Callback);
  ros::Subscriber sub3 = n.subscribe<std_msgs::Int8>("action/int8", 1, rl_action_Callback);
	ros::Subscriber sub4 = n.subscribe("slam_out_pose",1000, PoseUpdate);
	ros::Subscriber twist_sub5 = n.subscribe("/cmd_vel", 1, msgCallback);                //Subscriber for the topic "/cmd_vel", "/action/int8" to operate the motor
	ros::Subscriber sub6 = n.subscribe("mark", 1, marker_Callback);
	ros::Subscriber sub7 = n.subscribe<std_msgs::String>("catdog/String", 1, catdog_cnn_Callback);
	ros::Subscriber sub8 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, lidar_Callback);
	ros::Publisher pub_goal=n.advertise<core_msgs::goal_sw>("/goal_sw", 1);
	c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);
     if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
         printf("Failed to connect\n");
         close(c_socket);
         return -1;
     }

    while(ros::ok){

						if (sw==0){
				cout<<"start"<<endl;
				data[0] = -k2*(cmdvelx/0.057-cmdvely*1/0.057);
				data[1] = +k2*(cmdvelx/0.057+cmdvely*1/0.057);
				data[2] = -k2*(cmdvelx/0.057+cmdvely*1/0.057);
				data[3] = +k2*(cmdvelx/0.057-cmdvely*1/0.057);
				for (int i=0; i<5; i++){
						write(c_socket, data, sizeof(data));
				    ros::Duration(0.025).sleep();
				    }
				//scout<<"countball"<<count_num<<endl;
				if(abs(posex)>2.1){
					if (sw==0){
					sw=1;
					after_slam_goforward=1;
				}
				}
			}
			if (sw==1){
				if (after_slam_goforward==1){
					go_front(40,15);
					after_slam_goforward=2;
					cout<<"sw1start"<<endl;
				}
				data[4]=1;
				if (count2>0){
					t3=(z+50)*50/465;
					if ((z<300) and (x<50) and (-50<x)){
						if ( c==1){
							data[16]=1;
						}
						else if (c==0){
							data[16]=0;
						}
						go_front(30,t3);
						cout<<"eat mode"<<endl;
						countball++;
					}
					else if (x>50){
						turn_right(3,5);
						cout<<"eat right"<<endl;
					}
					else if(x<-50){
						turn_left(3,5);
						cout<<"eat left"<<endl;
					}
					else if (z >300){
						go_front(30,10);
						cout << "eat go"<<endl;
					}
				}
				else if(count_num==0){
					go_del(0,0,-0.7,5);
					cout <<count2 << "countnum" << count_num << endl;
					cout<<"nosee->turnleft"<<endl;
				}
				else{
					if (action == 0){
							go_del(1.0,0,0,5);
							cout<<"action1"<<endl;
						} // forward
						else if (action == 3){
							go_del(-0.6,0,0,5);
							cout<<"action3"<<endl;
						} // backward
						else if (action == 1){ // turn left
							go_del(0,0,-0.4,5);
							cout<<"action1"<<endl;
						}
						else if (action == 2){ // turn right
							go_del(0,0,0.4,5);
							cout<<"action2"<<endl;
						}
						else{
							go_del(0,0,0,5);
						}
						std::cout << "action" << action<<std::endl;
				}

			if (countball>=6){
				// pub_goal.publish(g_msg);
				// pub_goal.publish(g_msg);
				// pub_goal.publish(g_msg);
				// pub_goal.publish(g_msg);
				sw=2;
				}
			}

			else if (sw==2){
				data[4]=1;
				if (ssw==0){
					if(posez>0 and posez<0.996){
						turn_left(10,10);
					}
					else if(posez<0 and posez>-0.996){
						turn_right(10,10);
					}
					else{
						ssw=1;
						take_sleep(80);
						cout<<"ssw1 end"<<endl;
					}
				}
				else if(ssw==1){
					if (trans1==0){
						move_trans(posey,1.2,26);
						trans1=1;
					}
					else{
						if (posey>1.30){
							go_left(10,40);
							cout<<"goleft posey:"<<posey<<endl;
						}
						else if(posey<1.18){
							go_right(10,40);
							cout<<"goright posey:"<<posey<<endl;
						}
						else{
							dataInit();
							ssw=2;
						}
					}
				}
				else if(ssw==2){
					if(posez>0 and posez<0.996){
						turn_left(5,10);
					}
					else if(posez<0 and posez>-0.996){
						turn_right(5,10);
					}
					else{
						ssw=3;
						take_sleep(80);
					}
				}
					else if(ssw==3){
						if (posex>1.75){
							go_front(30,5);
							cout<<"ssw=2 go front"<<endl;
						}
						else{
							take_sleep(80);
							ssw=4;
						}
					}
					else if(ssw==4){
						if (trans2==0){
							move_trans(posey,0,26);
							trans2=1;
						}
						else{
							if (posey>0.20){
								go_left(10,20);
								cout<<"goleft posey:"<<posey<<endl;
							}
							else if(posey<-0.20){
								go_right(10,20);
								cout<<"goright posey:"<<posey<<endl;
							}
							else{
								dataInit();
								ssw=5;
							}
						}
					}
					else if(ssw==5){
						if (posex>0.1){
							go_front(30,5);
							cout<<"ssw=5 go front"<<endl;
						}
						else{
							take_sleep(20);
							ssw=6;
						}
					}
					else if(ssw==6){
						turn_left(40,42);
						take_sleep(20);
						ssw=7;
					}
					else if(ssw==7){
						go_front(40,40);
						sw=3;
					}
			}
			else if (sw==3){
				dataInit();
				cout << "sw == 3"<<endl;
				if(posez<0.498){
					turn_left(10,10);
					}
				else if(posez>0.502){
					turn_right(10,10);
					}
				if ( x_1 !=0 and y_1!=0){
					if (x_1+x_2 > c1)
					{
						nomarkersee=0;
				   	go_right(10,10);
						cout << "move right state"<<endl;
					}
					else if (x_1+x_2 < -c1)
					{
						nomarkersee=0;
				    go_left(10,10);
						cout<<"move left state"<<endl;
					}
					else if(x_1+x_2>-c1 and x_1+x_2<c2)
					{
						nomarkersee=0;
						if (y_1-y_2 > c2)
						{
				    		turn_left(10,5);
								cout << "rotate ccw state"<<endl;
						}
						else if (y_2-y_1 > c2)
						{
				    		turn_right(10,5);
							cout<<"rotate cw state"<<endl;
						}
						else
						{
							if (z_1+z_2 > c3)
							{
				    		go_front(10,5);
								cout<<"go forward stae"<<endl;
							}
							else if (z_1+z_2<c3)
							{
								cout << "z_1   "<<z_1 << "z_2" <<z_2 << "    x_1"<<endl;
								data[4] = 2;
								if (catordog==1){
									data[16]=1;
									secondrelease =0;
								}
								else if(catordog==2){
									data[16]=0;
									secondrelease =0;
								}
								for (int i=0; i<200; i++){
									write(c_socket, data, sizeof(data));
									ros::Duration(0.025).sleep();
								}
								cout<<"release state"<<endl;

								if (secondrelease==0){
									go_back(40,80);
									turn_left(40,84);
									go_front(40,40);
									secondrelease=1;

							}
						}
					}
					}
				}
				else{
					int kt=0;
					kt=kt+1;
					if (kt>5){
						turn_left(5,5);
						kt=0;
					}
				}
			}
			ros::spinOnce();
		}
		return 0;
	}
