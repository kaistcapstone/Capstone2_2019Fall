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
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include <core_msgs/ball_collect.h>                                        //For msg of the topic "/action/int8"
#include "core_msgs/ball_position_2.h"
#include "core_msgs/ball_position.h"
#include <algorithm>
//For motor keyboard control
#include "kbhit.h"
#include "getch.h"
float lu_k, ru_k, ld_k, rd_k =0.0;
float lu_r, ru_r, ld_r, rd_r=0.0;
float gap =0.1;
//end
int DQN_ON = 0;

int start_pickup = 0;
float Sxy = 0.2935;
float rad_factor = 0.0343;

int port;                                                                        //Port number for TCP/IP
std::string ip_address;                                                          //Address for TCP/IP
int path_fail=0;
double max_linear_vel;                                                           //Maximaum linear/angular velocity
double max_angular_vel;                                                          //Match thease with myRIO's corresponding parmameters

int c_socket;
struct sockaddr_in c_addr;
float data[24];
ros::Time Last_dqn;                                                               //Last time when the topic for DQN is recieved
bool Dqn_start;                                                                  //True if DQN is running
int sel_mov = 0;

using namespace std;

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
    //data[20] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);
    //data[21] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);
    data[22] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
    data[23] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);
}

static int release_start = 0;

void msgCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)                      //Receive topic /cmd_vel and write data[24] to send via TCP/IP
{
    if(DQN_ON != 1 && (release_start == 0 || release_start == -1) && (start_pickup == 0)){
      dataInit();
      int plumix;
      int plumiz;
      int case_num = 0;
      float linvel = 0;
      float angvel = 0;
      float transvel = 0;
      linvel = (cmd_vel->linear.x);
      angvel = (cmd_vel->angular.z);
      transvel = (cmd_vel->linear.y);
      // linvel = 0;
      // angvel = 0;
      // transvel = 0.1;

      int lin_sign = 1;
      int ang_sign = 1;
      float lin_fac=1.0;
      float rpm_limit = 6/(3.141592*0.0285);

      //Original Code
   //-++-
      float lu, ld, ru, rd;
      lu = (linvel - transvel - Sxy*angvel)/rad_factor*30/3.141592;
      ru = (linvel + transvel + Sxy*angvel)/rad_factor*30/3.141592;
      ld = (linvel + transvel - Sxy*angvel)/rad_factor*30/3.141592;
      rd = (linvel - transvel + Sxy*angvel)/rad_factor*30/3.141592;

      float wmax = std::max(std::max(abs(lu),abs(ru)),std::max(abs(rd),abs(ld)));
      if (wmax>rpm_limit){
        lu = lu*rpm_limit/wmax;
        ru = ru*rpm_limit/wmax;
        ld = ld*rpm_limit/wmax;
        rd = rd*rpm_limit/wmax;
      }

      data[0]=lu/90;
      data[1]=ru/90;
      data[4]=ld/90;
      data[5]=rd/90;
        // cout<<"lu:"<<data[0]<<"\n";
        // cout<<"ru:"<<data[1]<<"\n";
        // cout<<"ld:"<<data[4]<<"\n";
        // cout<<"rd:"<<data[5]<<"\n";
      cout<<"linear vel: "<<linvel<<"\n";
      cout<<"angular vel: "<<angvel<<"\n";
    }


    //Original Code end

    //Motor Control by Keyboard
    //
    // if(kbhit())
    // {
    //   int ch=getch();
    //   if(ch=='q')
    //   {
    //     lu_k=lu_k+gap;
    //     cout<<"lu : "<<lu_k<<endl;
    //   }
    //   else if(ch=='w')
    //   {
    //     lu_k=lu_k-gap;
    //     cout<<"lu : "<<lu_k<<endl;
    //   }
    //   else if(ch=='e')
    //   {
    //     ru_k=ru_k+gap;
    //     cout<<"ru : "<<ru_k<<endl;
    //   }
    //   else if(ch=='r')
    //   {
    //     ru_k=ru_k-gap;
    //     cout<<"ru : "<<ru_k<<endl;
    //   }
    //   else if(ch=='a')
    //   {
    //     ld_k=ld_k+gap;
    //     cout<<"ld : "<<ld_k<<endl;
    //   }
    //   else if(ch=='s')
    //   {
    //     ld_k=ld_k-gap;
    //     cout<<"ld : "<<ld_k<<endl;
    //   }
    //   else if(ch=='d')
    //   {
    //     rd_k=rd_k+gap;
    //     cout<<"rd : "<<rd_k<<endl;
    //   }
    //   else if(ch=='f')
    //   {
    //     rd_k=rd_k-gap;
    //     cout<<"rd : "<<rd_k<<endl;
    //   }
    //   else if(ch=='z') //spin clockwise
    //   {
    //     lu_k=lu_k+gap;
    //     ru_k=ru_k-gap;
    //     ld_k=ld_k+gap;
    //     rd_k=rd_k-gap;
    //     cout<<"CW spin : "<<rd_k<<endl;
    //   }
    //   else if(ch=='x') //spin counter clockwise
    //   {
    //     lu_k=lu_k-gap;
    //     ru_k=ru_k+gap;
    //     ld_k=ld_k-gap;
    //     rd_k=rd_k+gap;
    //     cout<<"CCW spin : "<<rd_k<<endl;
    //   }
    //   else if(ch=='k')
    //   {
    //     cout<<"lu_k : "<<lu_k<<endl;
    //     cout<<"ru_k : "<<ru_k<<endl;
    //     cout<<"ld_k: "<<ld_k<<endl;
    //     cout<<"rd_k : "<<rd_k<<endl;
    //   }
    //   else if(ch=='l')
    //   {
    //     cout<<"lu_r : "<<lu_r<<endl;
    //     cout<<"ru_r : "<<ru_r<<endl;
    //     cout<<"ld_r : "<<ld_r<<endl;
    //     cout<<"rd_r : "<<rd_r<<endl;
    //   }
    //   else if(ch=='o')
    //   {
    //     lu_r=lu_k;
    //     ru_r=ru_k;
    //     ld_r=ld_k;
    //     rd_r=rd_k;
    //   }
    //   else if(ch=='p')
    //   {
    //     lu_r=0.0;
    //     ru_r=0.0;
    //     ld_r=0.0;
    //     rd_r=0.0;
    //   }
    // }
    // data[0]=lu_r;
    // data[1]=ru_r;
    // data[4]=ld_r;
    // data[5]=rd_r;
    //KeyboardMotorcontrol Endr



    // if (cmd_vel->linear.x > 0){
    //     data[1] = 0.5;                                                                //Right stick's y-axis move. RU in LabVIEW
    // }else if(cmd_vel->linear.x < 0){
    //     data[1] = -0.5;
    // }
    // if (std::abs(cmd_vel->linear.x) <= max_angular_vel)
    //     data[1] = (cmd_vel->linear.x/max_linear_vel)*0.5;                                   //Left sticks's y-axis move. Joy1 Y-axis in LabVIEW;
    //
    // if(abs(cmd_vel->angular.z)>0.5)
    // {
    //   //data[1]=0;
    //   if (cmd_vel->angular.z > 0){
    //       data[4] = 0.5;                                                                  //LB in LabVIEW
    //   }else if(cmd_vel->angular.z < 0){
    //       data[4] = 0.5;
    //   }
    //   if (std::abs(cmd_vel->angular.z) <= max_angular_vel){
    //     data[4] = cmd_vel->angular.z/max_angular_vel;
    //   }
    //
    // }


    //ROS_INFO("linear_vel.x: %.4f  augular_vel.z: %.4f", cmd_vel->linear.x, cmd_vel->angular.z);
}
void dqn_intCallback(const std_msgs::Int8::ConstPtr& msg_int)
{
  DQN_ON = msg_int->data ;
}

int seen_ball = 0;

void ball_numCallback(const core_msgs::ball_position::ConstPtr& msg)
{
  seen_ball = msg->size;
}

float acc=0;
int picked_ball;

void dqnCallback(const std_msgs::Int8::ConstPtr& msg)                                  //Receive topic /action/int8 and write data[24] to send via TCP/IP
{
    if (start_pickup>0)
    {
      cout<<"pickup_start"<<endl;
    }
    int dqn_switch = msg->data;
    if(dqn_switch>-1 && start_pickup == 0){
      dataInit();
      // if (picked_ball == 5 && seen_ball == 0)
      // {
      //   dqn_switch = 1;
      //   acc=0;
      // }

      if (seen_ball == 0)
      {
        dqn_switch = 1;
        acc=0;
      }


      switch (dqn_switch){
          case 0: // Forward
            data[0] = 0.2;
            data[1] = 0.2;
            data[4] = 0.2;
            data[5] = 0.2; //Button_Y
            acc=0;
            break;
          // case 1: // Forward + Right
          //   data[0] = 0.3;
          //   data[5] = 0.3;
          //   break;
          // case 1: // Right
          //   data[0] = 0.2;
          //   data[1] = -0.2;
          //   data[4] = -0.2;
          //   data[5] = 0.2;
          //   break;
          // case 3: // Backward + Right
          //   data[1] = -0.3;
          //   data[4] = -0.3;
          //   break;
          // case 4: // Backward
          //   data[0] = -0.3;
          //   data[1] = -0.3;
          //   data[4] = -0.3;
          //   data[5] = -0.3;
          //   break;
          // case 5: // Backward + Left
          //   data[0] = -0.3;
          //   data[5] = -0.3;
          //   break;
          // case 2: // Left
          //   data[0] = -0.2;
          //   data[1] = 0.2;
          //   data[4] = 0.2;
          //   data[5] = -0.2;
          //   break;
          // case 7: // Left + Forward
          //   data[1] = 0.3;
          //   data[4] = 0.3;
          //   break;
          case 1:
            data[0] = -0.05-acc;
            data[1] = 0.05+acc;
            data[4] = -0.05-acc;
            data[5] = 0.05+acc;
            acc=min(acc+0.01 , 0.1);
            break;
          case 2:
            data[0] = 0.05+acc;
            data[1] = -0.05-acc;
            data[4] = 0.05+acc;
            data[5] = -0.05-acc;
            acc=min(acc+0.01 , 0.1);
            break;
          default:
              ROS_INFO("NOT VALID action/int8, out of range (0~7)");
          break;
      }
    }

    ROS_INFO("picked_ball : %d", picked_ball);
    ROS_INFO("Seen ball : %d", seen_ball);
    ROS_INFO("DQN INPUT ACTION: %d", dqn_switch);
    Last_dqn = ros::Time::now();                                                            //Save the time
    if (Dqn_start == 0) Dqn_start = 1;
                                                       //DQN is running since now
}


int picked_ball_prev = 0;
int suc_shutdown = 1;
void countCallback(const std_msgs::Int32::ConstPtr& msg){
  picked_ball = msg->data;
  ROS_INFO("picked ball cCb: %d",picked_ball);
}


void collectCallback(const core_msgs::ball_collect::ConstPtr& msg){
  start_pickup = msg->pick_start;
  if (start_pickup ==1  && picked_ball<6){
    dataInit();
    data[0] = msg->data00;
    data[1] = msg->data01;
    data[2] = msg->data02;
    data[3] = msg->data03;
    data[4] = msg->data04;
    data[5] = msg->data05;
    data[6] = msg->data06;
    data[7] = msg->data07;
    data[8] = msg->data08;
    data[9] = msg->data09;
    data[10] = msg->data10;
    data[11] = msg->data11;
    data[12] = msg->data12;
    data[13] = msg->data13;
    data[14] = msg->data14;
    data[15] = msg->data15;
    data[16] = (msg->data16) *suc_shutdown;
    data[17] = msg->data17;
    data[18] = msg->data18;
    data[19] = msg->data19;
    data[20] = msg->data20;
    data[21] = msg->data21;
    data[22] = msg->data22;
    data[23] = msg->data23;
    ROS_INFO("collect Callback");
  }
  if (start_pickup ==2) {
    data[0] = msg->data00;
    data[1] = msg->data01;
    data[2] = msg->data02;
    data[3] = msg->data03;
    data[4] = msg->data04;
    data[5] = msg->data05;
    data[6] = msg->data06;
    data[7] = msg->data07;
    data[8] = msg->data08;
    data[9] = msg->data09;
    data[10] = msg->data10;
    data[11] = msg->data11;
    data[12] = msg->data12;
    data[13] = msg->data13;
    data[14] = msg->data14;
    data[15] = msg->data15;
    data[16] = (msg->data16) * suc_shutdown;
    data[17] = msg->data17;
    data[18] = msg->data18;
    data[19] = msg->data19;
    data[20] = msg->data20;
    data[21] = msg->data21;
    data[22] = msg->data22;
    data[23] = msg->data23;
    cout<<"data16: "<< data[16]<<"\n"<<endl;
  //  ros::Duration(5).sleep();
  }
}



void relstartCallback(const std_msgs::Int32::ConstPtr&msg){
  release_start = msg->data;
  ROS_INFO("release_start : %d",release_start);
}

void releaseCallback(const core_msgs::ball_collect::ConstPtr& msg){
  if (release_start==1){
    data[0] = msg->data00;
    data[1] = msg->data01;
    data[2] = msg->data02;
    data[3] = msg->data03;
    data[4] = msg->data04;
    data[5] = msg->data05;
    data[6] = msg->data06;
    data[7] = msg->data07;
    data[8] = msg->data08;
    data[9] = msg->data09;
    data[10] = msg->data10;
    data[11] = msg->data11;
    data[12] = msg->data12;
    data[13] = msg->data13;
    data[14] = msg->data14;
    data[15] = msg->data15;
    data[16] = (msg->data16) * suc_shutdown;
    data[17] = msg->data17;
    data[18] = msg->data18;
    data[19] = msg->data19;
    data[20] = msg->data20;
    data[21] = msg->data21;
    data[22] = msg->data22;
    data[23] = msg->data23;
    ROS_INFO("release Callback");
  }
}

void ballpicked(const std_msgs::Int32::ConstPtr msg){
  if(msg -> data  == 1){
    suc_shutdown = 0;
  }
  else{
    suc_shutdown = 1;
  }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_sub");
    ros::NodeHandle nh_("~");

    nh_.param("ip_address", ip_address, std::string("172.16.0.1"));                          //TCP/IP target IP address. default : 172.16.0.1
    nh_.param("port", port, 4000);                                                           //TCP/IP target port number. default : 6001
    nh_.param("max_linear_vel", max_linear_vel, 1.0);                                        //Maximum linear, angular velocity of motor
    nh_.param("max_angular_vel", max_angular_vel, 1.0);

    ROS_INFO("ip_address: %s", ip_address.c_str());
    ROS_INFO("port: %d", port);
    ROS_INFO("max_linear_vel: %.2f m/s", max_linear_vel);
    ROS_INFO("max_angular_vel: %.2f rad/s", max_angular_vel);

    ros::Subscriber twist_sub_ = nh_.subscribe("/cmd_vel", 1, msgCallback);                //Subscriber for the topic "/cmd_vel", "/action/int8" to operate the motor
    ros::Subscriber dqn_sub_ = nh_.subscribe("/action/int8", 1, dqnCallback);
    ros::Subscriber ball_sub = nh_.subscribe("/collect_order", 10, collectCallback);
    ros::Subscriber dqn_sub_int = nh_.subscribe("/action_tf/int8", 1, dqn_intCallback);
    ros::Subscriber release_sub = nh_.subscribe("/release_step",1,releaseCallback);
    ros::Subscriber restart_sub = nh_.subscribe("/release_start",100,relstartCallback);
    ros::Subscriber arduino_sub = nh_.subscribe("/ball_check",1,ballpicked);
    ros::Subscriber ball_num_sub = nh_.subscribe("/position", 1, ball_numCallback);
    ros::Subscriber count_sub = nh_.subscribe("/picked_balls",1,countCallback);
    fd_set fdset;                                                                             //For time-out of socket
    struct timeval tv;

    c_socket = socket(PF_INET, SOCK_STREAM, 0);                                               //Create the socket for TCP/IP
    fcntl(c_socket, F_SETFL, O_NONBLOCK);
    c_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(port);

    connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr));                             //Try to connect

    FD_ZERO(&fdset);
    FD_SET(c_socket, &fdset);
    tv.tv_sec = 5; tv.tv_usec = 0;                                                              //Set time-out seconds

    if (select(c_socket+1, NULL, &fdset, NULL, &tv) == 1)                                       //True if socket is connected on time
    {                                                                                           //otherwise, terminates the node
        int so_error;
        socklen_t len = sizeof so_error;
        getsockopt(c_socket, SOL_SOCKET, SO_ERROR, &so_error, &len);
        if (so_error == 0){
            printf("Connected. %s:%d is open.\n", ip_address.c_str(), port);
        }
    }
    else
    {
        ROS_INFO("Fail to connect");
        nh_.shutdown();
        return -1;
    }

    while(ros::ok()){
        write(c_socket, data, sizeof(data));                                                      //Send the data via TCP/IP
        ros::Duration(1).sleep();                                                                 //Wait for 1 sec
        ros::spinOnce();


        if(DQN_ON == 0){

        }

        else if(DQN_ON ==1){}

        else{}

        // if (ros::Time::now()-Last_dqn > ros::Duration(2.0) && Dqn_start){
        //     dataInit();
        //     write(c_socket, data, sizeof(data));
        //     ROS_INFO("Time out. It's been two seconds after the last topic came in.");
        //     ros::Duration(5).sleep();
        // }
    }

    close(c_socket);
    return 0;
}
