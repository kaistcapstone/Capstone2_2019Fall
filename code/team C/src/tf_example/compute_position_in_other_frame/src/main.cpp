#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

struct ball{   
	double x, y, z;
};//struct ball will have 3 variables x, y, and z. 

struct ball ball1; //declare ball1
struct ball ball2; //declare ball2

void callback(const visualization_msgs::Marker& msg){  //subscribe message from fake_ball_in_rviz node
	ball1.x = msg.points.at(0).x;
	ball1.y = msg.points.at(0).y;
	ball1.z = msg.points.at(0).z;
	ball2.x = msg.points.at(1).x;	
	ball2.y = msg.points.at(1).y;
	ball2.z = msg.points.at(1).z;
	//ROS_INFO("balls: %f %f %f %f %f %f", ball1.x, ball1.y, ball1.z, ball2.x, ball2.y, ball2.z);
}



int main(int argc, char** argv){
  ros::init(argc, argv, "compute_position_in_other_frame");  //init ROS node
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("balls", 1, callback);  //declare subscriber

  tf::TransformListener listener;  //declare tf listener
  ros::Rate r(10); // 10 hz
  double x, y, z, qx, qy, qz, qw;  //variables to save transformation variables. it is just for printing. 

  while (ros::ok()){
    tf::StampedTransform transform; //declare transform


    //about try&catch, read README.md file for detail 
    try{ 
         listener.lookupTransform("/world", "/camera_link", ros::Time(0), transform); //this command will receive transformation between "world frame" and "camera_link frame"
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    ros::spinOnce();  //spinOnce to receive ros message

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    z = transform.getOrigin().z();
    
    qx = transform.getRotation().x();
    qy = transform.getRotation().y();
    qz = transform.getRotation().z();
    qw = transform.getRotation().w();

    ROS_INFO("transform: %f %f %f %f %f %f %f", x, y, z, qx, qy, qz, qw);  //print the tf information between "world" and "camera_link"
    ROS_INFO("position of ball1 at camera_link frame: %f %f %f", ball1.x, ball1.y, ball1.z);  //print the position of ball 1 in "camera_link" frame

    tf::Vector3 output; //declare tf::Vector3. this output will have values of transformed position of ball1
    tf::Vector3 input(ball1.x, ball1.y, ball1.z); //declare tf::Vector3. this input will have values of position of ball1 before trnasformation
    output = transform*input; // apply transformation.

    ROS_INFO("position of ball1 at world frame: %f %f %f", output.x(), output.y(), output.z());   //print the position of transformed point
 
    ROS_INFO("position of ball2 at camera_link frame: %f %f %f", ball2.x, ball2.y, ball2.z);  //same process as the case of ball1
    input.setX(ball2.x);
    input.setY(ball2.y);
    input.setZ(ball2.z);

    output = transform*input;
    ROS_INFO("position of ball2 at world frame: %f %f %f", output.x(), output.y(), output.z());  



    r.sleep();
  }
  return 0;
};
