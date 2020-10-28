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


#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position_2.h"
#include "core_msgs/ball_position_modify.h"
#include "core_msgs/ball_collect.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"


#include "opencv2/opencv.hpp"


#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
#define IPADDR "172.16.0.1" // myRIO ipadress




using namespace std;

boost::mutex map_mutex;


static float t = 0.025;//duration time

// webcam2 global variables
int web2_red_number=0; //number of red balls
int web2_green_number = 0;//number of green balls
int web2_blue_number=0;//number of blue balls
float web2_red_Z_array[20];//x position array of red balls
float web2_red_X_array[20];//x position array of red balls
float web2_red_X = -100;//x position of red ball
float web2_red_Z = 100;

float web2_blue_X_array[20];//x position array of red balls
float web2_blue_Z_array[20];//x position array of blue balls
float web2_blue_X = -100;//x position of blue ball
float web2_blue_Z = 100;//x position of blue ball


//float web2_center=0.1;
float web2_center=-0.5;

float insurance =0;

bool insurance_tf = false;

int leftright = -1; //when left: 0 when right: 1 - value to decide whether the closest green ball is left or right at releasing part

int collection=0;// number of collected blue balls

int action;

float acc=0.1;//acceleration when moving

float suc=10;//variable to count suction time and 'collection'

int release_direction = 0;//variable to decide whether we would turn CW or CCW when looking for basket

int suction_switch=0;//variable to help avoid redundant counting of 'collection'

int c_socket, s_socket;
int len;
int n;
float data[24];
struct sockaddr_in c_addr;

static float data00 = 0; //lx*data[3];
static float data01 = 0; //ly*data[3];
static float data02 = 0; //GamepadStickAngle(_dev, STICK_LEFT);
static float data03 = 0; //GamepadStickLength(_dev, STICK_LEFT);
static float data04 = 0; //rx*data[7];
static float data05 = 0; //ry*data[7];
static float data06 = 0; //GamepadStickAngle(_dev, STICK_RIGHT);
static float data07 = 0; //GamepadStickLength(_dev, STICK_RIGHT);
static float data08 = 0; //GamepadTriggerLength(_dev, TRIGGER_LEFT);
static float data09 = 0; //GamepadTriggerLength(_dev, TRIGGER_RIGHT);
static float data10 = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_UP);
static float data11 = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
static float data12 = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
static float data13 = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
static float data14 = 0; //GamepadButtonDown(_dev, BUTTON_A);
static float data15 = 0; //GamepadButtonDown(_dev, BUTTON_B);
static float data16 = 0; //GamepadButtonDown(_dev, BUTTON_X);
static float data17 = 0; //GamepadButtonDown(_dev, BUTTON_Y);
static float data18 = 0; //GamepadButtonDown(_dev, BUTTON_BACK);
static float data19 = 0; //GamepadButtonDown(_dev, BUTTON_START);
static float data20 = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);//stick left
static float data21 = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);//stick right
static float data22 = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
static float data23 = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);
static int pick_start = 1;

void dataInit()
{
	data00 = 0; //lx*data[3];
	data01 = 0; //ly*data[3];
	data02 = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data03 = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data04 = 0; //rx*data[7];
	data05 = 0; //ry*data[7];
	data06 = 0; //GamepadStickAngle(_dev, STICK_RIGHT);
	data07 = 0; //GamepadStickLength(_dev, STICK_RIGHT);
	data08 = 0; //GamepadTriggerLength(_dev, TRIGGER_LEFT);
	data09 = 0; //GamepadTriggerLength(_dev, TRIGGER_RIGHT);
	data10 = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_UP);
	data11 = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
	data12 = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
	data13 = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
	data14 = 0; //GamepadButtonDown(_dev, BUTTON_A);
	data15 = 0; //GamepadButtonDown(_dev, BUTTON_B);
	data16 = 0; //GamepadButtonDown(_dev, BUTTON_X);
	data17 = 0; //GamepadButtonDown(_dev, BUTTON_Y);
	data18 = 0; //GamepadButtonDown(_dev, BUTTON_BACK);
	data19 = 0; //GamepadButtonDown(_dev, BUTTON_START);
	// data20 = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);//servo motor
	// data21 = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);//suction
	data22 = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
	data23 = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);
	pick_start = 0;
}

void move_forward(float v);
void turn_CW(float w);
void turn_CCW(float w);
void stop();

//dealing with ball informations from webcam2
void camera2_Callback(const core_msgs::ball_position_2::ConstPtr& position_modify2)
{
	map_mutex.lock();//do nothing until finishing function of camear2_Callback
	cout<<"callback 2 start"<<endl;
	//initializing x, y, z values of balls
	web2_red_X = -100;
	web2_red_Z = 100;
	web2_blue_X = -100;
	web2_blue_Z = 100;
	// assigning number of balls detected
	web2_red_number = position_modify2->r_size;
	web2_blue_number = position_modify2->b_size;

	cout<<"total number of red balls from webcam2:"<<web2_red_number<<endl;
	cout<<"total number of blue balls from webcam2:"<<web2_blue_number<<endl;

	// assigning x,z position of closest red ball
    	for(int i = 0; i < web2_red_number; i++)
    	{
        	web2_red_Z_array[i] = position_modify2->r_img_z[i];
		if(web2_red_Z > position_modify2->r_img_z[i]){
			web2_red_Z = position_modify2->r_img_z[i];
			web2_red_X = position_modify2->r_img_x[i];
		}
    	cout<<"web2 red circle("<<i<<"):z="<<web2_red_Z_array[i]<<", z="<<endl;			//[groupD] for check
    	}

	// assigning x,z position of closest blue ball
	for(int i = 0; i < web2_blue_number; i++)
	{
		web2_blue_Z_array[i] = position_modify2->b_img_z[i];
		if(web2_blue_Z > position_modify2->b_img_z[i]){
			web2_blue_X = position_modify2->b_img_x[i];
			web2_blue_Z = position_modify2->b_img_z[i];
		}
		cout<<"web2 blue circle("<<i<<"):z="<<web2_blue_Z_array[i]<<endl;			//[groupD] for check
	}

	//find max and min of web2_green_x when web2 green number > 2.
 	out<<"callback 2 end"<<endl;
	map_mutex.unlock();
}

void suction_check(){
	//activate suction motor for 1.5 secs
	if(suc<=1.5){
		data16=1;
	}
	else{
		data16=0;
	}
}

void sleep_count(float sleeprate){
	ros::Duration(sleeprate).sleep();
	suc = suc + sleeprate;
	cout<<"suc ="<<suc<<endl;
	cout<<"collection is "<<collection<<endl;
	cout<<"direction_release is "<<release_direction<<endl;
	//recognize a ball collected, 0.5sec after blue ball disappears from webcam2 with suction_switch on
	if(collection<2){
		if(0.6<=suc && suction_switch ==1){
			collection = collection + 1;
			suction_switch =0;
			cout <<"collected 2 balls"<<endl;
		}
	}
	else{
		if(0.1<=suc && suction_switch ==1){
			collection = collection + 1;
			suction_switch = 0;
			cout<<"collected a ball!"<<endl;
		}
	}
}



void pick_up_blue(){
	if(web2_blue_X > -2.0 && web2_blue_X < 2){
		suction_switch = 1;
	}//turn on suction_switch when entering pick_up step - in order to count ball collected
	suc = 0;//initializing suc in order to turn on suction motor

  	cout<<"void starting pickup"<<endl;

	//adjusting direction towards blue ball
	if(web2_blue_X>1.5+web2_center){//if x position of blue ball is over 1
		while(web2_blue_X>1+web2_center && web2_blue_number != 0){//turn CW until x positoin of blue ball is less than 0.6
			turn_CW(0.05);
			ros::spinOnce();
			ros::Duration(t).sleep();
	 	}
 	}
	else if(web2_blue_X<-1.5+web2_center){
		//if x position of blue ball is less than -1
		while(web2_blue_X<-1+web2_center && web2_blue_number != 0){\
			//turn CCW until x position of blue ball is over -0.6
			turn_CCW(0.05);
			ros::spinOnce();
			ros::Duration(t).sleep();
		}
	 }
	else{//go forward if blue ball is in the 5middle
	if(web2_blue_Z < 0.4) move_forward(0.05);
	else move_forward(0.1);
  	}
}


void pick_up_red(){
	if(web2_red_X > -2.0 && web2_red_X < 2.0){
		suction_switch = 1;
	}//turn on suction_switch when entering pick_up step - in order to count ball collected
  	suc = 0;//initializing suc in order to turn on suction motor

  	cout<<"void starting pickup"<<endl;

	//adjusting direction towards blue ball
	if(web2_red_X>1.5+web2_center){//if x position of blue ball is over 1.5
		while(web2_red_X>1+web2_center && web2_red_number != 0){//turn CW until x positoin of blue ball is less than 0.6
			turn_CW(0.05);
			ros::spinOnce();
			ros::Duration(t).sleep();
		}
	}
	else if(web2_red_X<-1.5+web2_center){
		//if x position of blue ball is less than -1.5
		while(web2_red_X<-1+web2_center && web2_red_number != 0){
			//turn CCW until x position of blue ball is over -0.6
			turn_CCW(0.05);
			ros::spinOnce();
			ros::Duration(t).sleep();
		}
	}
	else{
		//go forward if blue ball is in the middle
		if(web2_red_Z < 0.4) move_forward(0.05);
	 	else move_forward(0.1);
  }
}

static int ii = 0;
static int delay = 0;

void choose_pick(){ //choose a ball to pick, and change the state of sorting motor
	cout<<"choosing a ball to pick...";
	if (web2_blue_Z < web2_red_Z){
		cout<<"pick blue";
		if(web2_blue_Z < 0.25)
		{
			data20 = 1; //move stick left -> blue ball stores in right
			data21 = 0; 
		}
		pick_up_blue();
	}
	else{
		cout<<"pick red";
		if(web2_red_Z < 0.25)
		{
			data20 = 0; //move stick right -> red ball stores in left
			data21 = 1; 
		}
		pick_up_red();
	}
}

ros::Publisher pub;
core_msgs::ball_collect msg;

void exitCallback(const std_msgs::Int32::ConstPtr& msg){
	//
	if (msg->data > 5){
		ROS_INFO("ball_collect node process terminated");
		exit(0);
	}
}

int main(int argc, char **argv)
{
	data20 = 0;
	data21 = 0;
    	ros::init(argc, argv, "ball_collect_node");
	ros::NodeHandle n;
	pub = n.advertise<core_msgs::ball_collect>("/collect_order", 100); //setting publisher

	ros::Subscriber sub2 = n.subscribe<core_msgs::ball_position_2>("/position_modify2", 1, camera2_Callback);
	ros::Subscriber sub3 = n.subscribe<std_msgs::Int32>("/picked_balls",1,exitCallback);
	while(ros::ok){
		dataInit();
      		ros::spinOnce();
		if(web2_blue_number!=0||web2_red_number!=0){
			choose_pick();
		}
		else {
			//ros::Duration(1).sleep();
			stop();
		}
		ROS_INFO("%f, %f, %f, %f, %f", data00, data01, data04, data05,	data16);
		cout<<"end of while loop"<<endl;
	    	sleep_count(t);
	}
	cout<<"quit while loop"<<endl;
	return 0;
}

static float first_2 = 10;

void move_forward(float v){
	first_2 = 10;
	//move as speed v, when red ball is not close to the ball
	//accelerate til speed v
	data00 = v;
	data01 = v;
	data04 = v;
	data05 = v;
	suction_check();
	pick_start = 1;
	msg.data00 = data00;
	msg.data01 = data01;
	msg.data02 = data02;
	msg.data03 = data03;
	msg.data04 = data04;
	msg.data05 = data05;
	msg.data06 = data06;
	msg.data07 = data07;
	msg.data08 = data08;
	msg.data09 = data09;
	msg.data10 = data10;
	msg.data11 = data11;
	msg.data12 = data12;
	msg.data13 = data13;
	msg.data14 = data14;
	msg.data15 = data15;
	msg.data16 = data16;
	msg.data17 = data17;
	msg.data18 = data18;
	msg.data19 = data19;
	msg.data20 = data20;
	msg.data21 = data21;
	msg.data22 = data22;
	msg.data23 = data23;
	msg.pick_start = pick_start;
	cout<<"message input done"<<endl;
	pub.publish(msg);
 	cout<<"void move forward"<<endl;
	ROS_INFO("%f, %f, %f, %f, %f", data00, data01, data04, data05,	data21);
 }

void turn_CW(float w){
	first_2 = 10;
	//turn CW with speed of w
	data00 = w;
	data01 = -w;
	data04 = w;
	data05 = -w;
	pick_start = 1;
	suction_check();
	msg.data00 = data00;
	msg.data01 = data01;
	msg.data02 = data02;
	msg.data03 = data03;
	msg.data04 = data04;
	msg.data05 = data05;
	msg.data06 = data06;
	msg.data07 = data07;
	msg.data08 = data08;
	msg.data09 = data09;
	msg.data10 = data10;
	msg.data11 = data11;
	msg.data12 = data12;
	msg.data13 = data13;
	msg.data14 = data14;
	msg.data15 = data15;
	msg.data16 = data16;
	msg.data17 = data17;
	msg.data18 = data18;
	msg.data19 = data19;
	msg.data20 = data20;
	msg.data21 = data21;
	msg.data22 = data22;
	msg.data23 = data23;
	msg.pick_start = pick_start;
	cout<<"message input done"<<endl;
	pub.publish(msg);
	cout<<"void turn CW"<<endl;
	ROS_INFO("%f, %f, %f, %f, %f", data[0], data[1], data[4], data[5],	data[21]);
}

void turn_CCW(float w){
//turn CCW with speed of w
	first_2 = 10;
	data00 = -w;
	data01 = w;
	data04 = -w;
	data05 = w;
	pick_start = 1;
	suction_check();
	msg.data00 = data00;
	msg.data01 = data01;
	msg.data02 = data02;
	msg.data03 = data03;
	msg.data04 = data04;
	msg.data05 = data05;
	msg.data06 = data06;
	msg.data07 = data07;
	msg.data08 = data08;
	msg.data09 = data09;
	msg.data10 = data10;
	msg.data11 = data11;
	msg.data12 = data12;
	msg.data13 = data13;
	msg.data14 = data14;
	msg.data15 = data15;
	msg.data16 = data16;
	msg.data17 = data17;
	msg.data18 = data18;
	msg.data19 = data19;
	msg.data20 = data20;
	msg.data21 = data21;
	msg.data22 = data22;
	msg.data23 = data23;
	msg.pick_start = pick_start;
	cout<<"message input done"<<endl;
	pub.publish(msg);
	cout<<"void turn CCW"<<endl;
	ROS_INFO("%f, %f, %f, %f, %f", data00, data01, data04, data05,	data21);
}

void stop(){
	ROS_INFO("stop is running");
	//turn CCW with speed of w
	data00 = 0;
	data01 = 0;
	data04 = 0;
	data05 = 0;
	suction_check();
	msg.data00 = data00;
	msg.data01 = data01;
	msg.data02 = data02;
	msg.data03 = data03;
	msg.data04 = data04;
	msg.data05 = data05;
	msg.data06 = data06;
	msg.data07 = data07;
	msg.data08 = data08;
	msg.data09 = data09;
	msg.data10 = data10;
	msg.data11 = data11;
	msg.data12 = data12;
	msg.data13 = data13;
	msg.data14 = data14;
	msg.data15 = data15;
	msg.data16 = data16;
	msg.data17 = data17;
	msg.data18 = data18;
	msg.data19 = data19;
	msg.data20 = data20;
	msg.data21 = data21;
	msg.data22 = data22;
	msg.data23 = data23;

	if (first_2 > 0){
			msg.pick_start = 2;
			first_2 = first_2 - 0.1;
	}
	else {
		msg.pick_start = 0;
	}

	pub.publish(msg);

	ROS_INFO("%f, %f, %f, %f, %f", data00, data01, data04, data05,	data21);
}
