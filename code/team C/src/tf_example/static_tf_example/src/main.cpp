#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "static_tf_example");  //init ROS node
  ros::NodeHandle nh;
  ros::Rate r(10);  //10 hz
  static tf::TransformBroadcaster br;  //set transformbroadcaster (similar to publisher)

  while(ros::ok()){

	tf::Transform transform;  //define transform
	transform.setOrigin( tf::Vector3(0.1, 0.1, 1.0) );  //set the translation related variables
	tf::Quaternion q;
	q.setRPY(1.57, 0, 0);  //set the rotation related variables here
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_link"));  //publish tf. parent link is "world" and child link is "camera_link"
	r.sleep();
  }



  return 0;
}

