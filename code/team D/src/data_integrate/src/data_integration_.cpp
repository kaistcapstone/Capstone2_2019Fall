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

#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
#define IPADDR "172.16.0.1"// myRIO ipadress

boost::mutex map_mutex;

int lidar_size; float lidar_degree[400]; float lidar_distance[400];
float lidar_obs; float k2=3.0; // constant
float k3=5; float c1 = 100;
float c2 = 15; float c3 = 80; //int ball_number; //float ball_X[20]; //float ball_Y[20];
//float ball_distance[20]; //int near_ball;
int count_num=0;
int action;
int	sw = 0;
int sw2 = 0;
int after_slam_goforward=0;
int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
int straight;
int t3;
int countball=0;
int rotat90=0;
int secondrelease=0;
float data[24];


int eatball=0;
#define RAD2DEG(x) ((x)*180./M_PI)
/*
sw 1  : first slam
sw 2  : dqn

*/
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
void turn_left(float size){
	data[0] = size; //lx*data[3];
	data[1] = size; //ly*data[3];
	data[2] = size; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = size; //GamepadStickLength(_dev, STICK_LEFT);
}
void turn_right(float size){
	data[0] = -size; //lx*data[3];
	data[1] = -size; //ly*data[3];
	data[2] = -size; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = -size; //GamepadStickLength(_dev, STICK_LEFT);
}
void go_front(float size){
	data[0] = -size; //lx*data[3];
	data[1] = +size; //ly*data[3];
	data[2] = -size; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = +size; //GamepadStickLength(_dev, STICK_LEFT);
}
void go_back(float size){
	data[0] = size; //lx*data[3];
	data[1] = -size; //ly*data[3];
	data[2] = size; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = -size; //GamepadStickLength(_dev, STICK_LEFT);
}
void go_del(float del_x, float del_y, float rot){

    data[0] = -k3*(del_x+del_y+rot*0.35);
    data[1] = +k3*(del_x-del_y-rot*0.35);
    data[2] = -k3*(del_x-del_y+rot*0.35);
    data[3] = +k3*(del_x+del_y-rot*0.35);
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


void webcam_Callback(const core_msgs::ball_bottom::ConstPtr& position)
{
	//std::cout<<"webcamcallback"<<std::endl;
  if (sw != 1){
  	//cout << "webcam out" <<endl;
  	return;
  	}
	t3=0;
	map_mutex.lock();
    int count2 = position->size;
    count_num += count2;
    int closest_idx = closest_ball(position-> img_z);
    if ( position ->size != 0){
    int x =position->img_x[closest_idx];
    int z =position->img_z[closest_idx];
    int c = position->color[closest_idx];
    t3=(z+50)*50/465;
    if (z<450){
    	eatball=1;
    	if ((z<300) and (x<50) and (-50<x)){
			//data[4]=1;
			go_front(30);
	    	cout<<"collectmode"<<endl;
	    	string text;
	    	if ( c==1){
				data[16]=1;
				text = "eat red";
				}
			else if (c==0){
				data[16]=0;
	    		text = "eat blue";
			}
			cout <<text << endl;
			for (int i=0; i<t3; i++){
	    //data[4]=1;
	    		write(c_socket, data, sizeof(data));
					cout <<data[0] << "\t"<<data[1]<<endl;

	    		ros::Duration(0.025).sleep();
	  		}
	  		//dataInit();
	  		data[4]=1;
	  		for (int i=0; i<40; i++){
	  			write(c_socket, data, sizeof(data));
					cout <<data[0] << "\t"<<data[1]<<endl;

	    		ros::Duration(0.025).sleep();
	  		}
	  		cout <<"end collectmode"<<endl;
	    //map_mutex.unlock();
	  		eatball=0;
	    	countball++;
		}
		else if (x>50){
				turn_right(3);
				for (int i=0; i<5; i++){
	  			write(c_socket, data, sizeof(data));
					cout <<data[0] << "\t"<<data[1]<<endl;

	    		ros::Duration(0.025).sleep();
	  		}
				cout<<"eat right"<<endl;
		}
		else if(x<-50){
				turn_left(3);
				for (int i=0; i<5; i++){
	  			write(c_socket, data, sizeof(data));
					cout <<data[0] << "\t"<<data[1]<<endl;

	    		ros::Duration(0.025).sleep();
	  		}
				cout<<"eat left"<<endl;
				}
		else if (z >300){
				go_front(10);
				for (int i=0; i<5; i++){
	  			write(c_socket, data, sizeof(data));
					cout <<data[0] << "\t"<<data[1]<<endl;

	    		ros::Duration(0.025).sleep();
	  		}
				cout << "eat go"<<endl;
		}

			}
	}
	map_mutex.unlock();
}

void rgbd_ball_Callback(const core_msgs::ball_position::ConstPtr& position)
{
    if (sw != 1){
		return;
		}
		count_num += position->size;
}

void rl_action_Callback(const std_msgs::Int8::ConstPtr& msg)
{
    if (sw != 1 or eatball==1){
		return;
		}
    cout<<"countball:"<<countball<<endl;
		map_mutex.lock();
		cout<<"rl_action state"<<endl;
    action = msg->data;

		data[4]=1;
    if (count_num ==0){
	    go_del(0,0,-0.3);
		cout<<"nosee->turnleft"<<endl;
	 	}
	else{
	    if (action == 0){
	    	go_del(0.4,0,0);

	    } // forward
	    else if (action == 3){
	    	go_del(-0.4,0,0);
	    } // backward
	    else if (action == 1){ // turn left
	        go_del(0,0,-0.3);
	    }
	    else if (action == 2){ // turn right
			go_del(0,0,0.3);
	    }
	    else{
			go_del(0,0,0);
	    }
	    std::cout << "action          " << action<<std::endl;
	//for (int i=0; i<5; i++){
	//  	}
	}

//		std::cout << "첫바퀴속도" << data[0]<<std::endl;
		//	std::cout<<"count : "<<count_num <<std::endl;
		count_num = 0;
		map_mutex.unlock();
}



void catdog_cnn_Callback(const std_msgs::String::ConstPtr& msg)
{
	int t4=10; // for 180 degree rotation
	int t5=10; // for moving backward
	int catordog=1;
		//map_mutex.lock();
	printf("cat");
  std::string catdog = msg->data;
		//map_mutex.unlock();
	printf("subscribe message is %s\n",catdog.c_str());
	if (catdog == "cat")
	{
		data[6] = 1;
		data[4] = 2;
		catordog=2;
		data[16] =1;
		cout<<"cat red"<<endl;
	}
	else
	{
		data[6] = 2;
		data[4] = 2;
		catordog=1;
		data[16] =0;
		cout<<"dog blue"<<endl;
	}
}

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
		//map_mutex.lock();
    int count = scan->scan_time / scan->time_increment;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        lidar_distance[i]=scan->ranges[i];
    }	//map_mutex.unlock();

}
void marker_Callback(const core_msgs::mark::ConstPtr& msg){
	if (sw !=4){
		return;
	}

	float x1, y1, z1, x2, y2, z2;
		//map_mutex.lock();
	cout<<"marker"<<endl;
  x1 = msg->x1;
	y1 = msg->y1;
	z1 = msg->z1;
	x2 = msg->x2;
	y2 = msg->y2;
	z2 = msg->z2;
		//map_mutex.unlock();
  dataInit();
	int del_x = 0;
	int del_y = 0;
	int rot = 0;
	if (x1+x2 > c1)
	{
    	go_del(0,1,0);
		printf("move right state");
	}
	else if (x1+x2 < -c1)
	{
    	go_del(0,-1,0);
		printf("move left state");
	}
	else
	{
		if (y1-y2 > c2)
		{
    		go_del(0,0,1);
			printf("rotate ccw state");
		}
		else if (y2-y1 > c2)
		{
    		go_del(0,0,-1);
			printf("rotate cw state");
		}
		else
		{
			if (z1+z2 > c3)
			{
    			go_del(-1,0,0);
				printf("go foward stae");
			}
			else
			{
				data[4] = 2;
				printf("release state");
				if (secondrelease=0){
				secondrelease=1;
				}
			}

		}
	}
	printf("%f",z1);
	printf("%f",z2);

//std::cout << "첫바퀴속도" << data[0]<<std::endl;
}
void align(float z, float vel){
	float ang=0;
	int t=0;
	dataInit();
	cout << "align"<<endl;
	if (z>0){
		ang = 180 - 180*z;
		t = ang * 17.5 / 15 ;
		turn_left(vel);
	}
	else {
		ang = 180 +180*z;
		t = ang * 17.5 / 15 ;
		turn_right(vel);
	}
	for (int i=0; i<t; i++){
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
	dataInit();
}
void move_trans (float y, float vel){
	int t=0;
	cout <<"move_trans" << endl;
	if ( y < 0.8){
		//go_right
		go_del(0,1 /26 *vel ,0 );
		t= (0.8-y) * 100/0.63;
	}
	else {
		 //go_ left
		go_del(0,-1 /26 *vel,0);
		t= (y-0.8) * 100/0.76;
	}
	for (int i=0; i<t; i++){
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
	dataInit();
}
void go_forward(float x, float goal, float vel){
	int t=0;
	cout << "go_forward" << endl;
	if ( x > goal){
		go_front(vel);
		t = (x-goal) * 50 / 0.465;
		if (vel == 50) t = (x-goal) * 100 / 1.435;
		for (int i=0; i<t; i++){
			write(c_socket, data, sizeof(data));
			ros::Duration(0.025).sleep();
		}
		dataInit();
	}
}
void msgCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)                      //Receive topic /cmd_vel and write data[24] to send via TCP/IP
{
	if (sw!=0)
		{cout<<"cmdout"<<endl;
	return;
	}
	cout<<"cmd_vel"<<endl;
	//go_del ( 2.7 * cmd_vel->linear.x, 2.7 * cmd_vel->linear.y,0);
  data[0] = -k2*(cmd_vel->linear.x/0.057-cmd_vel->linear.y*1/0.057);
  data[1] = +k2*(cmd_vel->linear.x/0.057+cmd_vel->linear.y*1/0.057);
  data[2] = -k2*(cmd_vel->linear.x/0.057+cmd_vel->linear.y*1/0.057);
  data[3] = +k2*(cmd_vel->linear.x/0.057-cmd_vel->linear.y*1/0.057);
	data[16]=0;
	data[4]=1;
  	//for (int i=0; i<5; i++){
	//  			write(c_socket, data, sizeof(data));
	//    		ros::Duration(0.025).sleep();
	//  	}

}
void PoseUpdate(const geometry_msgs::PoseStampedConstPtr& pose)
{
   	if(sw!=0 and sw!=2 and sw!=3) {
		cout << "pose out" <<endl;
		return;
		}
   	geometry_msgs::PoseStampedConstPtr pose_ptr_;
   	pose_ptr_ = pose;
	float x = 	pose_ptr_->pose.position.x;
	float y =	pose_ptr_->pose.position.y;
	float z =   pose_ptr_->pose.orientation.z;
	if (sw==0){
	 	if(x>1.5 and abs(y)>0.75){
		sw=1;
		after_slam_goforward=1;
		cout<<"go"<<endl;
		}
	}

	float epsi=0.05;
	float k5=5;
	float k6=0;
	if (sw==2){
		align (z,30);
		move_trans(y, 30);
		align (z,30);
		float goal = (x-1.1)*0.3+1.1;
		go_forward(x, goal, 50);
		align(z,15);
		move_trans(y, 30);
		go_forward(x, 1.1, 30);
		}
	if (sw==3){
		if (x <0.3){
			sw=4;
		}
	}
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;
		//dataInit();
		printf("0");


    ros::Subscriber sub3 = n.subscribe<core_msgs::ball_position>("/position", 1, rgbd_ball_Callback);
   	ros::Subscriber sub78 = n.subscribe<core_msgs::ball_bottom>("/position2", 1, webcam_Callback);
   	ros::Subscriber sub4 = n.subscribe<std_msgs::Int8>("action/int8", 1, rl_action_Callback);
   	cout<< "switch test"<<endl;

 	//if (sw==0 or sw==2 or sw==3) {
   	   ros::Subscriber sub111 = n.subscribe("slam_out_pose",1, PoseUpdate);

 		//ros::Subscriber sub6 = n.subscribe<geometry_msgs::PolygonStamped>("/move_base/global_costmap/footprint", 1, footprint_Callback);
 		ros::Subscriber twist_sub = n.subscribe("/cmd_vel", 1, msgCallback);                //Subscriber for the topic "/cmd_vel", "/action/int8" to operate the motor
	//}
		std::cout <<"count"<<count_num<<std::endl;
		cout << "sw =" <<sw<<endl;
	//if (sw==4){
		ros::Subscriber sub2 = n.subscribe("mark", 1, marker_Callback);
		ros::Subscriber sub10 = n.subscribe<std_msgs::String>("catdog/String", 1, catdog_cnn_Callback);
	//}
	// Subscriber for the readmarker
    //ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 100, camera_Callback);
		//ros::Subscriber sub3 = n.subscribe<std_msgs::Int8>("action/int8", 1000, rl_action_Callback);
		//ros::Subscriber sub3 = n.subscribe<std_msgs::String>("catdog/String", 1000, catdog_cnn_Callback);
 	ros::Subscriber sub8 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, lidar_Callback);
	// Subscriber for the readmarker
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

	if ( eatball!=1){
	for (int i=0; i<5; i++){
			write(c_socket, data, sizeof(data));
			cout <<data[0] << "\t"<<data[1]<<endl;
		    ros::Duration(0.025).sleep();

		    }}

    //if( sw!=0){
	if (sw==1 and after_slam_goforward==1){
		go_front(40);
		cout<< "in_after go"<<endl;
		for (int i=0; i<25; i++){
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
	cout<< "after_slam_goforward" <<endl;
	after_slam_goforward =2;
	dataInit();
	}
	else if (sw==1 and countball>=6)
	{
		sw=2;
		dataInit();
		for (int i=0; i<80; i++){
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
		}
	}

	if (sw==3)
	{	//if (s2)return;
    	core_msgs::goal_sw goal;
		goal.data =1;
  		ros::NodeHandle nh;
    	ros::Publisher pub10 = nh.advertise<core_msgs::goal_sw>("/goal_sw", 1); //setting publisher
		std::cout<<"goal pub"<<std::endl;
		pub10.publish(goal);
	}
eatball=0;
	if (sw==4){
		data[17] =1;
cout<< "sw 4 in" <<endl;
		//if (rotat90==0){
		//	turn_left(50);
		//	for (int i=0; i<38; i++){
		 //   write(c_socket, data, sizeof(data));
		  //  ros::Duration(0.025).sleep();
		//	}
			rotat90=1;
		}
	}
	if (secondrelease==1){
		cout << "second in"<<endl;
		cout<<"secondrelease"<<endl;
		dataInit();
		data[4]=2;
		for (int i=0; i<120; i++){
			write(c_socket, data, sizeof(data));
			ros::Duration(0.025).sleep();
		}
		go_back(50);
		for (int i=0; i<80; i++){
			write(c_socket, data, sizeof(data));
			ros::Duration(0.025).sleep();
		}
		turn_left(50);
		for (int i=0; i<76; i++){
			write(c_socket, data, sizeof(data));
			ros::Duration(0.025).sleep();
		}
		secondrelease=2;
	}

	ros::spinOnce();

    }

    return 0;
}
