#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "fake_ball_in_rviz");   //init ROS node
	ros::NodeHandle nh;
	ros::Publisher pub;   //set up publisher
	pub = nh.advertise<visualization_msgs::Marker>("/balls",1);  //ball's position will be publisher in a form "visualization_msgs::Marker"
	ros::Rate r(10);  //10 hz

	while(ros::ok()){

		visualization_msgs::Marker ball_list;  //declare marker
		ball_list.header.frame_id = "/camera_link";  //set the frame
		ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
		ball_list.ns = "balls";   //name of markers
		ball_list.action = visualization_msgs::Marker::ADD;  
		ball_list.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw) 
		ball_list.pose.position.y=0;
		ball_list.pose.position.z=0;
		ball_list.pose.orientation.x=0;
		ball_list.pose.orientation.y=0;
		ball_list.pose.orientation.z=0;
		ball_list.pose.orientation.w=1.0;

		ball_list.id = 0; //set the marker id. if you use another markers, then make them use their own unique ids
		ball_list.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker

		ball_list.scale.x=0.05; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
		ball_list.scale.y=0.05;
		ball_list.scale.z=0.05;

		//set up the first ball
		geometry_msgs::Point p;
		p.x = 0.1;
		p.y = 0.1;
		p.z = 0.1;
		ball_list.points.push_back(p);

		std_msgs::ColorRGBA c;
		c.r = 0.0;
		c.g = 1.0;
		c.b = 0.0;
		c.a = 1.0;
		ball_list.colors.push_back(c);
		
		//second ball
		p.x = -0.1;
		p.y = -0.1;
		p.z = -0.1;
		ball_list.points.push_back(p);

		c.r = 1.0;
		c.g = 0.0;
		c.b = 0.0;
		c.a = 1.0;
		ball_list.colors.push_back(c);
		

		pub.publish(ball_list);  //publisher
		r.sleep();

	}

	return 0;
}
