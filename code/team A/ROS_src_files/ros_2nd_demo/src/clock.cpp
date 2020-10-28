#include "ros/ros.h"
#include "ros_2nd_demo/Message1.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clock");
  ros::NodeHandle nh;

  ros::Publisher ros_2nd_pub = nh.advertise<ros_2nd_demo::Message1>("time", 100);

  ros::Rate loop_rate(1000);

  ros_2nd_demo::Message1 msg;

  float count = 0;
  // int i = 0;
  while (ros::ok())
  {

    msg.data_1 = count;
    // ROS_INFO("send test = %d", i);
    ROS_INFO("send msg = %.3f", msg.data_1);

    ros_2nd_pub.publish(msg);
    count += 0.001;
    // i++;
    loop_rate.sleep();
  }
  return 0;
}
