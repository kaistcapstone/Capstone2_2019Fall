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
#include <ctime>


#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/ball_position_back.h"
#include "core_msgs/markerposition.h"
#include "core_msgs/pick.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"

#include "opencv2/opencv.hpp"

#define PI 3.141592
#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
#define IPADDR "172.16.0.1" // host computer ip

using namespace std;
int c_socket, s_socket;
struct sockaddr_in c_addr;

/*------------------------------------------------------------------------------------------
---------------------------------------stage number-----------------------------------------
--------------------------------------------------------------------------------------------
stage 0 (cmd_Callback) : SLAM을 통해 출발하여 미로를 탈출한다. find_frontier에서 end_flag를 보내면 다음 stage로 이동한다.
stage 1 (rl_action_Callback) : DQN을 이용하여 공을 수집한다. 공을 6개 모두 모으면 다음 stage로 이동한다.
stage 2 (cmd_Callback) : SLAM을 통해 얻는 현재 pose로 로봇을 시작 방향에서 180도 회전하고 시작 위치에서 2.65m~2.85m 앞의 방향으로 이동시킨다.
stage 3 (cmd_Callback) : SLAM을 통해 미로를 빠져나와 시작 위치로 이동한다.
stage 4 (main) : 로봇의 pose를 이용해 90도 회전시킨다. 로봇의 후면이 시작 위치에서 오른쪽 벽을 바라보게 된다.
stage 5 (main) : 90도 회전한 로봇의 후면 카메라는 마커를 바라본다. marker와 align을 하면서 후진한다.
stage 6 (main) : cat인지 dog인지 인식한다.
stage 7 (main) : 다시 marker와 align하면서 후진한다. 일정 거리에 도달하면, 개면 파란공, 고양이면 빨간공을 내보낸다.
stage 8 (main) : 일정거리 직진해서 다른 바구니로 접근한다. 
stage 9 (main) : 180도 회전한다.
stage 10 (main) : 180도 회전한 로봇의 후면 카메라는 다른 마커를 바라본다. marker와 align을 하면서 후진한다.
stage 11 (main) : cat인지 dog인지 인식한다.
stage 12 (main) : 다시 marker와 align하면서 후진한다. 일정 거리에 도달하면, 개면 파란공, 고양이면 빨간공을 내보낸다.
stage 13 (main) : 모든 임무를 완수했으므로 data_integration_pick_node를 종료한다.
---------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------*/

int motion_info; //dqn에서 받아오는 action을 저장하는 변수
int catdog=2; //cnn에서 받은 string자료형을 int형으로 저장하는데, 고양이라면 0이고 강아지면 1을 저장하는 변수
geometry_msgs::PoseStampedConstPtr pose_ptr_; //SLAM으로 받아오는 로봇의 현재 위치를 저장하는 변수
int IsPoseUpdated = 0; //null pointer error를 피하기 위해 만든 변수, pose_ptr_을 받을 수 있다면 1이 된다.

int ball_number; //rgbd카메라에 보이는 공의 갯수
float ball_X[20]; //공의 x 좌표를 저장하는 array
float ball_Y[20]; //공의 y 좌표를 저장하는 array
float closest_ball[2]; //가장 가까운 공의 좌표를 저장하는 array. closest_ball[0]은 x좌표, closest_ball[1]은 y좌표이다.

float data[24]; //myRio에 보내는 data 값들의 array

/*아래의 변수들은 검은색 마커의 위치를 저장하는 변수들이고, aver와 wid는 각각 위의 두점의 x 평균값, 마커 네점의 넓이
그리고 marker_key는 마커에 대한 정보를 메세지로 받고 있음을 확인하기 위한 변수이다. */
int marker_x1;
int marker_y1;
int marker_x2;
int marker_y2;
int marker_x3;
int marker_y3;
int marker_x4;
int marker_y4;
int marker_aver;
int marker_key;
float r=0;
int c=0;
int r1=0;
int c1=0;

int stage_number = 0; //로봇은 각 stage 마다 해야만 하는 과제를 가지고 있다. 
int catdog_f;
int picked_ball_number=0; //pickup한 파란 공의 개수
int picked_ball_number_R=0; //pickup한 빨간 공의 개수

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

/* realsense camera로부터 현재 보이는 공들의 위치를 message로 받아온다.
공의 위치를 바탕으로 로봇과 공을 align하는 과정에 사용된다.*/
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
    ball_number=position->size;
    closest_ball[0] = 10000;
    closest_ball[1] = 10000;
    for(int i = 0; i < count; i++)
    {
        ball_X[i] = position->img_x[i];
        ball_Y[i] = position->img_y[i];
        float ball_distances = ball_X[i] * ball_X[i] + ball_Y[i] * ball_Y[i];
        float closest_ball_distance = closest_ball[0] * closest_ball[0] + closest_ball[1] * closest_ball[1];
        if (ball_distances<closest_ball_distance){
          closest_ball[0] = ball_X[i];
          closest_ball[1] = ball_Y[i];
        }
    }
}

/* 공과 로봇을 align하는 함수다.
그러나 공을 주울 때, DQN외의 외부 간섭을 사용하지 않겠다고 결정했기 때문에 노드에서 사용되지 않는다. */ 
void ball_align(){
  int threshold_d = 0.05;
  if(closest_ball[0]>-0.02){
    cout<<"turn right"<<endl;
    data[9] = 0.5; //0.12
  }
  else if(closest_ball[0]<-0.19){
    data[9] = -0.5;
    cout<<"turn left"<<endl;
  }
  else{
    cout<<"go"<<endl;
    data[5] = 0.15;
  }
}

// pick_node로부터 m현재 주운 공의 갯수를 message로 받아온다.
void ball_Callback(const core_msgs::pick::ConstPtr& msg1){
    picked_ball_number = msg1->blue;
    picked_ball_number_R = msg1->red;
}

/* 로봇의 pose를 message로 받는 callback 함수이다.
로봇의 pose를 저장하고, IsPoseUpdated가 1이 된다.
IsPoseUpdated는 null pointer error를 없애기 위해 만들었다. */
void PoseUpdate(const geometry_msgs::PoseStampedConstPtr& pose)
{
   pose_ptr_ = pose;
   IsPoseUpdated = 1;
}

/* move_base가 제공하는 cmd_vel을 message로 받는 callback 함수다.
이 함수에서 stage 0, stage 2, stage 3이 진행된다. */
void cmd_Callback(const geometry_msgs::Twist::ConstPtr& msg){
  printf("cmd_Callback loop\n");
  if((stage_number==0)||(stage_number==3)){
    data[5] = (msg->linear.x);
    data[4] = -(msg->linear.y);
    data[9] = -(msg->angular.z);
    data[17] = 1;

    if(stage_number==3){
      printf("x: %f\n", data[5]);
      printf("y: %f\n", data[4]);
      printf("z: %f\n", data[9]);
      if(abs(pose_ptr_->pose.position.x) < 0.2){
        stage_number = 4;
        printf("stage_number: 4\n");
      }
      printf("stage_number = 3\n");
    }
    else if(stage_number == 0){
      printf("stage_number = 0\n");
      printf("x = %f\n", data[5]);
      printf("y = %f\n", data[4]);
      printf("w = %f\n", data[9]);
    }
  }
  if(stage_number==2){
    int angle_flag = 0;
    printf("stage_number = 2\n");
    dataInit();
    printf("%f\n", pose_ptr_->pose.orientation.w);
    if(abs(pose_ptr_->pose.orientation.w) < 0.2){
      angle_flag=1;
      dataInit();
    }
    else{
      if(pose_ptr_->pose.orientation.z>0){
          data[9] = -0.5; //ccw
      }
      else{
          data[9] = 0.5;
      }
    }
    if((pose_ptr_->pose.position.x)>2.85 && angle_flag==1){
      data[5]= 0.075;
    }
    else if((pose_ptr_->pose.position.x)<2.65 && angle_flag==1){
      data[5]= -0.075;
    }
    else if(rl_flag==1){
      dataInit();
      stage_number=3;
    }
  }
}

/* find_frontier에서 end_flag를 message로 받는 callback 함수이다.
end_flag를 받으면, stage 0을 종료하고 stage 1로 넘어간다. */
void end_Callback(const std_msgs::Int32::ConstPtr& msg){
  if(stage_number == 0){
    data[5] = 0;
    data[4] = 0;
    data[9] = 0;
    stage_number=1;
  }
}

/* DQN의 action을 message로 받는 callback 함수이다.
stage 1이 진행되는 함수다.
DQN이 학습되었던 simulation을 재현하기 위해, 각 action마다 delay를 넣었다. */
void rl_action_Callback(const std_msgs::Int8::ConstPtr& msg)
{
  motion_info = msg->data;
  if(stage_number == 1){
    dataInit();
    if (motion_info==0){
      data[5] = 0.15;
      data[9] = 0;
      write(c_socket, data, sizeof(data));
      printf("fw\n");
      int k2 = 0;
      while(k2 < 1){
        printf("ccw\n");
        ros::Duration(0.5).sleep();
      k2++;}
    }
    else if (motion_info==1){
      data[9] = -0.5; //-0.4
      data[5] = 0;
      write(c_socket, data, sizeof(data));
      printf("ccw\n");
      int k2 = 0;
      while(k2 < 1){
        printf("ccw\n");
        ros::Duration(0.31415).sleep();
      k2++;}
    }
    else if (motion_info==2){
      data[9] = 0.5;
      data[5] = 0;
      printf("cw\n");
      write(c_socket, data, sizeof(data));
      int k3 = 0;
      while(k3 < 1){
        printf("cw\n");
         ros::Duration(0.31416).sleep();
      k3++;}
    }
    if((picked_ball_number>2)&&(picked_ball_number_R>2)){
      printf("stage1 end\n");
      stage_number=2;
    }
  }
}

/* marker와 align하는 함수다.
x1 : 첫 번째 marker의 x 좌표
x2 : 두 번째 marker의 x 좌표
angle_z : 회전할 때, 목표로 하는 각도
threshold_x : x축 align시의 threshold
threshold_anlge : 회전할 때, 각도의 threshold
x축 align, y축 align, 각도 align 성공시 return 값(cost)에 1을 더한다.
하나만 성공시는 1, 둘은 2, 모두 성공시에는 3을 return한다.*/
int align(int x1, int x2, float angle_z, int threshold_x, float threshold_angle){
  int cost=0;
  if (IsPoseUpdated){
    float angle = pose_ptr_->pose.orientation.z;
    if(abs(angle-angle_z)>threshold_angle){
      if(angle>angle_z){
        data[9]=0.05;
      }
      else{
        data[9]=-0.05;
      }
    }
    else{
      data[9]=0;
      printf("y algin completed\n");
      cost++;
    }
  }
  if((abs((x1+x2)/2-320))>threshold_x){
    if((x1+x2)/2>320){
      data[4]=-0.03;
    }
    else{
      data[4]=0.02;
    }
  }
  else{
    data[4]=0;
    cost++;
    printf("x algin completed\n");
  }
  if(abs(x1-x2)<120){
    data[5]=-0.05;
  }
  else{
    data[5]=0;
    cost++;
    printf("backward completed\n");
  }
  printf("%d\n", cost);
  return cost;
}

void central_back(){
  dataInit();
  data[5]=-0.05;
  if(marker_aver > 310){
    data[4]=-0.05;
  }
  if(marker_aver < 290){
    data[4]=0.05;
  }
}

/* marker 4개의 좌표를 message로 받아오는 callback함수다.
4개의 좌표를 저장하고, 그 평균값을 계산한다. */
void marker_Callback(const core_msgs::markerposition::ConstPtr& msg){
  marker_x1=msg->x1;
  marker_y1=msg->y1;
  marker_x2=msg->x2;
  marker_y2=msg->y2;
  marker_x3=msg->x3;
  marker_y3=msg->y3;
  marker_x4=msg->x4;
  marker_y4=msg->y4;
  marker_aver = (marker_x1+marker_x2)/2;
  marker_key=1;
}

/* CNN으로부터 cat인지 dog인지 string으로 message를 받는 callback 함수이다.
정확한 구분을 위해서 cat인지 dog인지 총 10번 값을 받아온 뒤, 많은 쪽을 선택한다.
cat으로 9번, dog로 1번 값이 들어오면 cat을 선택하는 방식이다. */
void catdog_cnn_Callback(const std_msgs::String::ConstPtr& msg)
{
  if(!(strcmp(msg->data.c_str(), "cat"))){
    r=r+0;
    // catdog=0;
  }
  else{
    r=r+1;
    // catdog=1;
  }
  c++;
  if(c>9){
    if((r/10)>0.5){
      catdog=1;
      printf("dog\n\n\n\n");
    }else{
      catdog=0;
      printf("cat\n\n\n\n");
    }
    r=0;
    c=0;
    printf("reset\n\n\n\n");
  } 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_integration_pick_node");
  ros::NodeHandle nh;

  ros::Subscriber sub_slam = nh.subscribe("/cmd_vel", 1000, cmd_Callback);
  ros::Subscriber endsub = nh.subscribe("TheEnd", 1000, end_Callback);
  ros::Subscriber sub2 = nh.subscribe("picked_ball", 1000, ball_Callback);
  ros::Subscriber sub = nh.subscribe("marker_pixel", 1000, marker_Callback);
  ros::Subscriber sub_dqn = nh.subscribe<std_msgs::Int8>("action/int8", 1, rl_action_Callback);
  ros::Subscriber sub_posi = nh.subscribe("position123", 1, camera_Callback);
  ros::Subscriber sub_pose = nh.subscribe("slam_out_pose",1000, PoseUpdate);
  ros::Subscriber sub5 = nh.subscribe<std_msgs::String>("catdog/String", 1000, catdog_cnn_Callback);

  // ---------- tcp connection -------------
  c_socket = socket(PF_INET, SOCK_STREAM, 0);
  c_addr.sin_addr.s_addr = inet_addr(IPADDR);
  c_addr.sin_family = AF_INET;
  c_addr.sin_port = htons(PORT);
  if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
    printf("Failed to connect\n");
		close(c_socket);
		return -1;
  }
  // ---------- drvie start -------------

  while(ros::ok()){
    printf("stage_number = %d\n\n", stage_number);
    marker_key=0;
    ros::spinOnce();

    //로봇의 pose를 받아서 시작 위치에 왔다고 판단하면, stage 4로 남어간다.
    if(IsPoseUpdated&&(abs(pose_ptr_->pose.position.x) < 0.3)&&(stage_number==3)){
      stage_number=4;
    }

    if(stage_number>4){
      dataInit();
    }

    // stage 4 : SLAM에서 받아오는 로봇의 pose를 활용해서 처음 임무 시작 방향에서 90도만큼 회전한다.
    // 처음 방향에서 90도 회전했다고 pose를 받아오면, 다음 stage로 이동한다. 
    if(stage_number==4){
      if(IsPoseUpdated&&(pose_ptr_->pose.orientation.z > 0) && (abs(pose_ptr_->pose.orientation.w - 0.707) < 0.01)){
        dataInit();
        stage_number = 5;
      }
      else{
        data[5]=0;
        data[4]=0;
        data[9]=0.5;
      }
    }

    // stage 5 : 90도 회전한 로봇은 marker를 후면 카메라로 바라보게 되는데, marker들과 align하면서 후진한다.
    // 일정 범위에 들어오면 stage 6으로 넘어간다.
    if(stage_number ==5){
        if(align(marker_x1, marker_x2, 0.7, 1, 0.01)==3){
          stage_number=6;
        }
    }

    // stage 6 : marker로부터 일정 범위 떨어졌을 때, 개인지 고양이 사진인지 결정하고 변수로 저장한다.
    // 사진을 가장 잘 볼 수 있는 거리에서 결정하기 때문에 정확성이 높다.
    // CNN으로부터 10번 받은 data 중 최빈값으로 개인지 고양이인지 결정하기 때문에 정확성이 더 높다.
    if(stage_number == 6){
      if(catdog==1){
        catdog_f =1;
        stage_number = 7;
      }else{
        catdog_f=0;
        stage_number = 7;
      }
    }

    // stage 7 : 다시 marker와 align하면서 뒤로 후진한다.
    // 일정 거리에 도달할 때까지 후진하면, catdog에 따라서 빨간공을 내보낼지, 파란공을 내보낼지 결정해서 뒷문을 연다.
    if(stage_number == 7){
      dataInit();
      if(abs(marker_x1-marker_x2)>130){
        int q8=0;
        // 추가 후진 단계
        while(q8<1){
          data[5]=-0.05;
          write(c_socket, data, sizeof(data));
          ros::Duration(4.9).sleep();
          q8++;
        }
        dataInit();
        // 개인지 고양이인지 판별 후, 공 내보내기 단계
        if(catdog_f==1){
          data[14]=1;
          data[15]=0;
          write(c_socket, data, sizeof(data));
          int q4=0;
            while(q4<1){
              ros::Duration(3).sleep();
              q4++;
            }
          stage_number = 8;
        }else{
          data[14]=0;
          data[15]=1;
          write(c_socket, data, sizeof(data));
          int q5=0;
            while(q5<1){
              ros::Duration(3).sleep();
              q5++;
            }
          stage_number = 8;
        }
      }else{
        // marker를 보면서 일정 거리 후진 단계
        central_back();
      }
    }

    // stage 8 : 일정거리 앞으로 직진하여 다른 바구니로 접근한다.
    if(stage_number == 8){
      dataInit();
      int q=0;
      while(q<60){
        data[5]=0.15;
        write(c_socket, data, sizeof(data));
        ros::Duration(0.1).sleep();
        q++;
      }
      stage_number = 9;
    }
 
    // stage 9 : 로봇을 180도 회전시켜 후면 카메라로 다른 바구니의 marker를 바라보게 한다.
    if(stage_number == 9){
      dataInit();
      int q2=0;
      while(q2<9){
        data[9]=0.5;
        write(c_socket, data, sizeof(data));
        ros::Duration(1).sleep();
        q2++;
      }
      stage_number =10;
      dataInit();
    }

    // stage 10 : stage 5와 동일
    if(stage_number ==10){
        if(align(marker_x1, marker_x2, -0.7, 1, 0.01)==3){
          stage_number=11;
        }

    }

    // stage 11 : stage 6과 동일
    if(stage_number == 11){
      if(catdog==1){
        catdog_f=1;
        stage_number = 12;
      }else{
       catdog_f=0;
       stage_number = 12;
      }
    }


    // stage 12 : stage 7과 동일
    if(stage_number == 12){
      dataInit();
      if(abs(marker_x1-marker_x2)>130){
        int q7=0;
        while(q7<1){
          data[5]=-0.05;
          write(c_socket, data, sizeof(data));
          ros::Duration(4.9).sleep(); //0.003
          q7++;
        }
        dataInit();
        if(catdog_f==1){
          data[14]=1;
          data[15]=0;
          write(c_socket, data, sizeof(data));
          int q2=0;
          while(q2<1){
            ros::Duration(1).sleep();
            q2++;
          }
          stage_number = 13;
        }else{
          int q3=0;
          while(q3<40){
            data[14]=0;
            data[15]=1;
            write(c_socket, data, sizeof(data));
            ros::Duration(0.1).sleep();
            q3++;
          }
          stage_number = 13;
        }
      }else{
        central_back();
      }
    }

    // stage 13 : node를 종료한다.
    if(stage_number==13){
      break;
    }

    // myRIO에 data를 보낸다.
    write(c_socket, data, sizeof(data));
  }

  close(c_socket);
  return 0;
}
