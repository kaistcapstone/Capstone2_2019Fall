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
// #include <windows.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_collect.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "core_msgs/wall_distance.h"

#include "opencv2/opencv.hpp"


// #define RAD2DEG(x) ((x)*180./M_PI)

// #define PORT 4000
// #define IPADDR "172.16.0.1" // myRIO ipadress




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
float web2_center=0;

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

//dealing with ball informations from webcam2

std::string catdog;

void catdog_cnn_Callback(const std_msgs::String::ConstPtr& msg)
{
		map_mutex.lock();
    catdog = msg->data;
		map_mutex.unlock();
		printf("subscribe message is %s\n",catdog.c_str());

}


void move_left(float v);
void move_right(float v);
void move_forward(float v);
void turn_CW(float w);
void turn_CCW(float w);
void stop();
static int step = 0;
int marker_exist = 0;
int marker_x[8];
float marker_distance[8];

void step0(); // turn until marker exist == 2
void step1(); // go straight, and move to step 2 if readmarker image is close enough
void step2(); // make the robot vertical with the wall, and park the robot
void step3(); // check cat or dog, and release
void step4(); // go back about 1.3 meters slowly
void step5(); // turn 180 degrees until detecting next marker
void step6(); // go straight, do same thing with step 1
void step7(); // same with step 2
void step8(); // same with step 3, end.
void step9();
// int32 marker_exist
// int32[] marker_x
// float32[] marker_distance

void markerCallback(const core_msgs::wall_distance::ConstPtr& msg){
	marker_exist = msg->marker_exist;
	if (msg->marker_exist == 2){
		for(int i=0;i<8;i++){
			marker_x[i] = msg->marker_x[i];
			marker_distance[i] = msg->marker_distance[i];
		}
	}

	switch(step){
		case 0:
			step0();
			break;
		case 1:
			step1();
			break;
		case 2:
			step2();
			break;
		case 3:
			step3();
			break;
		case 4:
			step4();
			break;
		case 5:
			step5();
			break;
		case 6:
			step6();
			break;
		case 7:
			step7();
			break;
		case 8:
			step8();
			break;
		case 9:
			step9();
		break;
	}
}

void step0(){
	//cout << "step0 running\n";
	ROS_INFO("step0 running");
	cout<<"Marker exist: "<<marker_exist <<endl;
	if (marker_exist != 2) {
		turn_CCW(0.05);
	}
	else{
		step++;
	}

}

static int center = 640;
static int stop_time = 0;
static int thres = 150;
static int first_drop_place = 0;

void step1(){
	ROS_INFO("step1 running");
	if ((marker_distance[0] < 0.5 && marker_distance[0] != 0)||(marker_distance[4] < 0.5 && marker_distance[4] != 0)){
		step++;
	}
	if (marker_exist != 2){
		stop();
		stop_time++;
		if (stop_time>50){
			step --;
			stop_time = 0;
		}
	}
	else{
		if(marker_x[0] != 0){
			if ((marker_x[0]+marker_x[1]+marker_x[2]+marker_x[3])/4 > (center + thres)){
				turn_CW(0.1);
			}
			else if ((marker_x[0]+marker_x[1]+marker_x[2]+marker_x[3])/4 < (center - thres)){
				turn_CCW(0.1);
			}
			else {
				move_forward(0.3);
			}
			first_drop_place = 1234;
		}
		else if(marker_x[4] != 0){
			if ((marker_x[4]+marker_x[5]+marker_x[6]+marker_x[7])/4 > (center + thres)){
				turn_CW(0.1);
			}
			else if ((marker_x[4]+marker_x[5]+marker_x[6]+marker_x[7])/4 < (center - thres)){
				turn_CCW(0.1);
			}
			else {
				move_forward(0.3);
			}
			first_drop_place = 5678;
		}
	}
	cout<<"first drop: "<<first_drop_place<<endl;
}

void step2(){
	const float thres_rot = 0.05;
	ROS_INFO("step2 running");
	if(marker_exist == 2){
		int img_center = 0;
		for(int i = 0; i<8; i++){
			img_center = img_center + marker_x[i];
		}
		img_center = img_center/4;
		if((img_center - center) > thres){
			move_right(0.2);
		}
		else if((img_center - center) < -thres){
			move_left(0.2);
		}
		else{
			if(marker_x[0] != 0){
				if((marker_distance[0] - marker_distance[1]) > thres_rot){
					turn_CW(0.05);
				}
				else if((marker_distance[0] - marker_distance[1]) < -thres_rot){
					turn_CCW(0.05);
				}
				else{
					if((marker_distance[0]+marker_distance[1])/2>0.37){
						move_forward(0.1);
					}
					else{
						step++;
					}
				}

			}
			else if(marker_x[4] != 0){
				if((marker_distance[4] - marker_distance[5]) > thres_rot){
					turn_CW(0.05);
				}
				else if ((marker_distance[4] - marker_distance[5]) < -thres_rot){
					turn_CCW(0.05);
				}
				else{
					if((marker_distance[4]+marker_distance[5])/2>0.37){
						move_forward(0.1);
					}
					else{
						step++;
					}
				}
			}
		}
	}
	else{
		stop();
		stop_time++;
		if (stop_time>50){
			step = 0;
			stop_time = 0;
		}
	}
}


ros::Publisher pub;
core_msgs::ball_collect msg;
static int delay = 0;
void step3(){
	ROS_INFO("step3 running");
	dataInit();
	if (catdog == "cat"){
		data08 = 0;
		data09 = 1;
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
		msg.pick_start = 0;
		cout<<"message input done"<<endl;
		pub.publish(msg);
		// ros::Duration(1).sleep();
		delay++;
		if (delay > 15){
			step++;
			delay = 0;
		}
		//step++;
		// for(int i = 0; i<10000; i++){
		// 	int a = 1234*1234;
		// }
		//Sleep(1000);
	}
	else if (catdog == "dog"){
		data08 = 1;
		data09 = 0;
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
		msg.pick_start = 0;
		cout<<"message input done"<<endl;
		pub.publish(msg);
		// ros::Duration(1).sleep();
		delay++;
		if (delay > 20){
			step++;
			delay = 0;
		}
		// for(int i = 0; i<10000; i++){
		// 	int a = 1234*1234;
		// }
		//Sleep(1000);
		// step++;
	}
	else{
		ROS_INFO("ERROR: we can't identify whether it's cat or dog");
		step --;
	}

}

void step4(){
	ROS_INFO("step4 running");
	dataInit();
	msg.data00 = -0.1;
	msg.data01 = -0.1;
	msg.data02 = data02;
	msg.data03 = data03;
	msg.data04 = -0.1;
	msg.data05 = -0.1;
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
	msg.pick_start = 0;
	cout<<"message input done"<<endl;
	pub.publish(msg);
	ros::Duration(0.1).sleep();
	delay++;
	if (delay > 30){
		step++;
		delay = 0;
	}
}

void step5(){
	ROS_INFO("step5 running");
	if ((marker_distance[0] > 1.6)||(marker_distance[4] > 1.6)){
		step++;
	}

	if (marker_exist != 2){
		stop();
		stop_time++;
		if (stop_time>100){
			step++;
			stop_time = 0;
		}
	}

	else{
		if(marker_x[0] != 0){
			if ((marker_x[0]+marker_x[1]+marker_x[2]+marker_x[3])/4 > (center + thres)){
				turn_CW(0.05);
			}
			else if ((marker_x[0]+marker_x[1]+marker_x[2]+marker_x[3])/4 < (center - thres)){
				turn_CCW(0.05);
			}
			else {
				move_forward(-0.1);
			}
		}
		else if(marker_x[4] != 0){
			if ((marker_x[4]+marker_x[5]+marker_x[6]+marker_x[7])/4 > (center + thres)){
				turn_CW(0.05);
			}
			else if ((marker_x[4]+marker_x[5]+marker_x[6]+marker_x[7])/4 < (center - thres)){
				turn_CCW(0.05);
			}
			else {
				move_forward(-0.1);
			}
		}
	}
}

void step6(){
	ROS_INFO("step6 running");
	cout<<"first drop: "<<first_drop_place<<endl;
	if (marker_exist != 2) {
		turn_CW(0.05);
	}
	else{
		if(marker_x[0] != 0 && first_drop_place != 1234){
			step++;
		}
		else if(marker_x[4] != 0 && first_drop_place != 5678){
			step++;
		}
		else{
			turn_CW(0.05);
		}
	}
}

void step7(){
	ROS_INFO("step7 running");
	if (((marker_distance[0]+marker_distance[1])/2 < 0.5 && marker_distance[0] != 0)||((marker_distance[4]+marker_distance[5])/2 < 0.5 && marker_distance[4] != 0)){
		step++;
	}
	if (marker_exist != 2){
		stop();
		stop_time++;
		if (stop_time>50){
			step --;
			stop_time = 0;
		}
	}
	else{
		if(marker_x[0] != 0){
			if ((marker_x[0]+marker_x[1]+marker_x[2]+marker_x[3])/4 > (center + thres)){
				turn_CW(0.05);
			}
			else if ((marker_x[0]+marker_x[1]+marker_x[2]+marker_x[3])/4 < (center - thres)){
				turn_CCW(0.05);
			}
			else {
				move_forward(0.3);
			}
		}
		else if(marker_x[4] != 0){
			if ((marker_x[4]+marker_x[5]+marker_x[6]+marker_x[7])/4 > (center + thres)){
				turn_CW(0.05);
			}
			else if ((marker_x[4]+marker_x[5]+marker_x[6]+marker_x[7])/4 < (center - thres)){
				turn_CCW(0.05);
			}
			else {
				move_forward(0.3);
			}
		}
	}
}

void step8(){
	const float thres_rot = 0.03;
	ROS_INFO("step8 running");
	if(marker_exist == 2){
		int img_center = 0;
		for(int i = 0; i<8; i++){
			img_center = img_center + marker_x[i];
		}
		img_center = img_center/4;
		if((img_center - center) > thres){
			move_right(0.2);

		}
		else if((img_center - center) < -thres){
			move_left(0.2);
		}
		else{
			if(marker_x[0] != 0){
				// if((marker_distance[0] - marker_distance[1]) > thres_rot){
				// 	turn_CW(0.05);
				// }
				// else if((marker_distance[0] - marker_distance[1]) < -thres_rot){
				// 	turn_CCW(0.05);
				// }
				// else{
					if((marker_distance[0]+marker_distance[1])/2>0.37){
						move_forward(0.1);
					}
					else{
						step++;
					}
				// }
			}
			else if(marker_x[4] != 0){
				// if((marker_distance[4] - marker_distance[5]) > thres_rot){
				// 	turn_CW(0.05);
				// }
				// else if ((marker_distance[4] - marker_distance[5]) < -thres_rot){
				// 	turn_CCW(0.05);
				// }
				// else{
					if((marker_distance[4]+marker_distance[5])/2>0.37){
						move_forward(0.1);
					}
					else{
						step++;
					}
				// }
			}
		}
	}
	else{
		stop();
		stop_time++;
		if (stop_time>50){
			step = 5;
			stop_time = 0;
		}
	}
}

void step9(){
	ROS_INFO("step9 running");
	dataInit();
	if (catdog == "cat"){
		data08 = 0;
		data09 = 1;
	}
	else if (catdog == "dog"){
		data08 = 1;
		data09 = 0;
	}
	else ROS_INFO("ERROR: we can't identify whether it's cat or dog");
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
	msg.pick_start = 0;
	cout<<"message input done"<<endl;
	pub.publish(msg);
	ros::Duration(1).sleep();
	cout << "The End";
}

int main(int argc, char **argv)
{
		data20 = 0;
		data21 = 0;
    ros::init(argc, argv, "ball_release_node");
		ros::NodeHandle n;
		pub = n.advertise<core_msgs::ball_collect>("/release_step", 100); //setting publisher


    //ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub2 = n.subscribe<core_msgs::wall_distance>("/marker_info", 1, markerCallback);
		ros::Subscriber sub3 = n.subscribe<std_msgs::String>("catdog/String", 1, catdog_cnn_Callback);

		while(ros::ok){
			// [groupD]
			//map_mutex.lock();
			dataInit();
      ros::spinOnce();
     	//map_mutex.unlock()
			// if (step == 3){
			// 	for(int i =0;i<2;i++){
			// 		ros::Duration(3).sleep();
			// 		ros::spinOnce();
			// 	}
			// }
			//ROS_INFO("%f, %f, %f, %f, %f", data00, data01, data04, data05,	data16);
			//cout<<"end of while loop"<<endl;
	    ros::Duration(0.05).sleep();
	 	}
		 //ROS_INFO("%f, %f, %f, %f, %f", data[0], data[1], data[4], data[5],	data[21]);  // for exp
     //write(c_socket, data, sizeof(data));
    cout<<"quit while loop"<<endl;

    return 0;
}


void move_forward(float v){
	 //move as speed v, when red ball is not close to the ball
	 //accelerate til speed v
	if(data01 < v){
		 data00 = data00+acc;
		 data01 = data01+acc;
		 data04 = data04+acc;
		 data05 = data05+acc;
   }
 	else if(data01 > v){
		 data00 = data00-acc;
		 data01 = data01-acc;
		 data04 = data04-acc;
		 data05 = data05-acc;
   }
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
	msg.pick_start = 0;
	cout<<"message input done"<<endl;
	pub.publish(msg);
  cout<<"void move forward"<<endl;
	//ROS_INFO("%f, %f, %f, %f, %f", data00, data01, data04, data05,	data21);
 }

void turn_CW(float w){
 //turn CW with speed of w
 data00 = w;
 data01 = -w;
 data04 = w;
 data05 = -w;
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
 msg.pick_start = 0;
 cout<<"message input done"<<endl;
 pub.publish(msg);
 cout<<"void turn CW"<<endl;
 //ROS_INFO("%f, %f, %f, %f, %f", data[0], data[1], data[4], data[5],	data[21]);
}

void turn_CCW(float w){
	//turn CCW with speed of w
	data00 = -w;
	data01 = w;
	data04 = -w;
	data05 = w;
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
	msg.pick_start = 0;
	cout<<"message input done"<<endl;
	pub.publish(msg);
	cout<<"void turn CCW"<<endl;
	//ROS_INFO("%f, %f, %f, %f, %f", data00, data01, data04, data05,	data21);
}

void stop(){
	//turn CCW with speed of w
	data00 = 0;
	data01 = 0;
	data04 = 0;
	data05 = 0;
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
	msg.pick_start = 2;
	cout<<"message input done"<<endl;
	pub.publish(msg);
	cout<<"void stop"<<endl;
	//ROS_INFO("%f, %f, %f, %f, %f", data00, data01, data04, data05,	data21);
}


void move_left(float v){
	//move left, accelerate til speed 0.8
	if(data[0]>-v){
	data00 = data00-acc;
	data01 = data01+acc;
	data04 = data04+acc;
	data05 = data05-acc;
  }
	cout<<"void move left"<<endl;
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
  msg.pick_start = 0;
  cout<<"message input done"<<endl;
  pub.publish(msg);
	//ROS_INFO("%f, %f, %f, %f", data00, data01, data04, data05);
}

void move_right(float v){
	//move right, accelerate til speed 0.8
	if(data[0]<v){
	data00 = data00+acc;
	data01 = data01-acc;
	data04 = data04-acc;
	data05 = data05+acc;
	}
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
  msg.pick_start = 0;
  cout<<"message input done"<<endl;
  pub.publish(msg);
	cout<<"void move right"<<endl;
	//ROS_INFO("%f, %f, %f, %f, %f", data00, data01, data04, data05,	data06);
}
