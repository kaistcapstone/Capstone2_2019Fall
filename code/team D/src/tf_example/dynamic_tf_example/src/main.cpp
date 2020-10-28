#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "dynamic_tf_example");
  ros::NodeHandle nh;
  ros::Rate r(10);
  static tf::TransformBroadcaster br;

  double pos = 0.1;
  double rot = 0;

  double delta_pos = 0.01;

  while(ros::ok()){

	if(pos<0.1){
		delta_pos = 0.01;
	}
	else if(pos>1.0){
		delta_pos = -0.01;
	}
	pos = pos + delta_pos;
	rot = rot + 3.14/10;


	tf::Transform transform;
	transform.setOrigin( tf::Vector3(pos,pos,pos+0.5) );
	tf::Quaternion q;
	q.setRPY(0, 0, rot);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_link"));
	r.sleep();
  }



  return 0;
}

