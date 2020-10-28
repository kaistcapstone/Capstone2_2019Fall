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
float lidar_obs; float k2=3.7; // constant
float k3=26; float c1 = 14;
float c2 = 15; float c3 = 700; //int ball_number; //float ball_X[20]; //float ball_Y[20];
//float ball_distance[20]; //int near_ball;
int count_num=0;
int action;
int	sw = 0;
int ssw =0;
int sw2 = 0;
int c=0;
int x=0;
int z=0;
int slep=0;
int x_1;
int x_2;
int y_1;
int y_2;
int z_1;
int z_2;
int ssww=0;
int sssw=0;
int after_slam_goforward=0;
int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
int straight;
int t3;
int bomb=0;
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
	int closest_idx = closest_ball(position-> img_z);
	if ( position ->size != 0){
	 x =position->img_x[closest_idx];
	 z =position->img_z[closest_idx];
	 c = position->color[closest_idx];
cout << "x" <<x <<"   z"<<z <<"     c"<<c<<endl;
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
}
void catdog_cnn_Callback(const std_msgs::String::ConstPtr& msg){
	std::string catdog = msg->data;
	cout<<"catdogcallback"<<endl;
	if (catdog=="cat"){
		catordog=1;
		cout<<"Itiscat"<<endl;
	}
	else if (catdog=="dog"){
		catordog=2;
		cout<<"Itisdog"<<endl;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_integation");
  ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1, rgbd_ball_Callback);
  ros::Subscriber sub2 = n.subscribe<core_msgs::ball_bottom>("/position2", 1, webcam_Callback);
  ros::Subscriber sub3 = n.subscribe<std_msgs::Int8>("action/int8", 1, rl_action_Callback);
	ros::Subscriber sub4 = n.subscribe("slam_out_pose",1000, PoseUpdate);
	ros::Subscriber twist_sub5 = n.subscribe("/cmd_vel", 1, msgCallback);                //Subscriber for the topic "/cmd_vel", "/action/int8" to operate the motor
	ros::Subscriber sub6 = n.subscribe("mark", 1, marker_Callback);
	ros::Subscriber sub7 = n.subscribe<std_msgs::String>("catdog/String", 1, catdog_cnn_Callback);
	ros::Subscriber sub8 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, lidar_Callback);
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
				if(abs(posex)>1.9){
					if (sw==0){
					sw=1;
					after_slam_goforward=2;
					}
				}
			}
			else if (sw==1){
				if (after_slam_goforward==1){
					go_front(40,15);
					after_slam_goforward=2;
					cout<<"sw1start"<<endl;
				}
				data[4]=1;
				if (count2>0 and z<300){
					bomb=0;
					t3=((z+100)*50/465);
					if ((z<250) and (x<50) and (-50<x)){
						if ( c==1){
							data[16]=1;
						}
						else if (c==2){
							data[16]=0;
						}
						go_front(30,t3);
						cout<<"eat mode"<<endl;
						countball++;
						take_sleep(20);
					}
					else if (x>50){
						turn_right(3,5);
						cout<<"eat right"<<endl;
					}
					else if(x<-50){
						turn_left(3,5);
						cout<<"eat left"<<endl;
					}
					else if (z >250){
						go_front(12,5);
						cout << "eat go"<<endl;
					}
				}
				else if(count_num==0){
					go_del(0,0,-0.7,5);
					bomb=bomb+1;
					if(bomb>200){
						sw=2;
					}
				}
				else{
					bomb=0;
					if (action == 0){
							go_del(1.0,0,0,5);
							cout<<"action0"<<endl;
						} // forward
						else if (action == 3){
							go_del(-0.5,0,0,5);
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
				sw=2;
				}
			}
			else if (sw==2){
				data[4]=1;
				if (ssw==0){
					if(posez>0 and posez<0.996){
						turn_left(15,5);
					}
					else if(posez<0 and posez>-0.996){
						turn_right(15,5);
					}
					else{
						ssw=1;
						take_sleep(20);
						cout<<"ssw1 end"<<endl;
					}
				}
				else if(ssw==1){
					if (trans1==0){
						if(posey>1.22){
							int t5;
							t5=100/76*(100*posey-125)*3/5;
							go_left(50,t5);
						}
						else{
							int t6;
							t6=100/70*(125-100*posey)*3/5;
							go_right(50,t6);
						}
						trans1=1;
					}
					else{
						if (posey>1.22){
							go_left(30,10);
							cout<<"goleft posey:"<<posey<<endl;
						}
						else if(posey<1.10){
							go_right(30,10);
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
						take_sleep(20);
					}
				}
					else if(ssw==3){
						if (posex>1.52){
							go_front(50,5);
							cout<<"ssw=3 go front"<<endl;
						}
						else{
							take_sleep(20);
							ssw=4;
						}
					}
					else if(ssw==4){
						if (trans2==0){
							if(posey>0){
								int t5;
								t5=100/76*(100*posey)*3/5;
								go_left(50,t5);
							}
							else{
								int t6;
								t6=100/70*(0-100*posey)*3/5;
								go_right(50,t6);
							}
							trans2=1;
						}
						else{
							if (posey>0.20){
								go_left(20,10);
								cout<<"goleft posey:"<<posey<<endl;
							}
							else if(posey<-0.20){
								go_right(20,10);
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
							go_front(40,5);
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
						go_front(50,40);
						sw=3;
					}
			}
			else if (sw==3){
				dataInit();
				if (slep==0){
					take_sleep(160);
					slep=1;
				}
				else{
				if ( x_1 !=0 and y_1!=0){
					cout << "bef rotate"<<endl;
					if (y_1-y_2 > c2)
					{
							turn_right(5,5);
							cout << "rotate ccw state"<<endl;
					}
					else if (y_2-y_1 > c2)
					{
							turn_left(5,5);
						cout<<"rotate cw state"<<endl;
					}
					else {
						if (x_1+x_2 > c1)
						{
					   	go_right(5,5);
							cout << "move right state" << x_1 <<"\t"<< y_1<<endl;
						}
						else if (x_1+x_2 < -c1)
						{
					    go_left(5,5);
							cout<<"move left state"<<endl;
						}
						else
						{
							if (y_1+y_2 > c3)
							{
				    		go_front(20,5);
								cout<<"go forward stae"<<endl;
							}
							else
							{
								cout << "z_1   "<<z_1 << "z_2" <<z_2 << "    x_1"<<endl;
								data[4] = 2;
								data[17]=1;
								if (sssw==0){
									if (catordog==1){
										data[16]=1;
										secondrelease =0;
										ssww=0;
									}
									else if(catordog==2){
										data[16]=0;
										secondrelease =0;
										ssww=1;
									}
									sssw=1;
								}
								else if (sssw==1){
									if (ssww==0){
										data[16]=0;
									}
									else{
										data[16]=1;
									}
								}
								if (secondrelease==1){
									sw=4;
								}
								for (int i=0; i<200; i++){
									write(c_socket, data, sizeof(data));
									ros::Duration(0.025).sleep();
								}
								cout<<"release state"<<endl;
								if (secondrelease==0){
									go_back(40,80);
									turn_left(40,84);
									go_front(40,50);
									secondrelease=1;
							}
						}
					}
					}
				}
			}
			}
			ros::spinOnce();
		}
		return 0;
	}
