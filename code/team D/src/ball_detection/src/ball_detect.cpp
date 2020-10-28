#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

using namespace cv;
using namespace std;

float intrinsic_data[9] = {615.6702685546875, 0.0, 328.001, 0.0, 615.962, 241.310858154297, 0.0, 0.0, 1.0};
float distortion_data[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

vector<float> pixel2point(Point center, int radius);
Mat buffer(320,240,CV_8UC1);
Mat buffer2(320,240,CV_16UC1);
ros::Publisher pub;
ros::Publisher pub_markers;
string intToString(int n);
string floatToString(float f);


int car_height=300;



Mat hsv_frame_blue, hsv_frame_red;
//int low_h_b=90, low_s_b=120, low_v_b=70;
//int high_h_b=113, high_s_b=255, high_v_b=255;
// void on_low_h_thresh_trackbar_red_1(int, void *);
// void on_high_h_thresh_trackbar_red_1(int, void *);
// void on_low_h2_thresh_trackbar_red_1(int, void *);
// void on_high_h2_thresh_trackbar_red_1(int, void *);
// void on_low_s_thresh_trackbar_red_1(int, void *);
// void on_high_s_thresh_trackbar_red_1(int, void *);
// void on_low_v_thresh_trackbar_red_1(int, void *);
// void on_high_v_thresh_trackbar_red_1(int, void *);
int low_h_r_1=160, high_h_r_1=180, low_s_r_1=135, low_v_r_1=115;
int low_h_r_2=0, high_h_r_2=15;

int high_s_r_1=255, high_v_r_1=255;
// void on_low_h_thresh_trackbar_blue_1(int, void *);
// void on_high_h_thresh_trackbar_blue_1(int, void *);
// void on_low_s_thresh_trackbar_blue_1(int, void *);
// void on_high_s_thresh_trackbar_blue_1(int, void *);
// void on_low_v_thresh_trackbar_blue_1(int, void *);
// void on_high_v_thresh_trackbar_blue_1(int, void *);
int low_h_b=70, low_s_b=180, low_v_b=50;
int high_h_b=130, high_s_b=255, high_v_b=255;

void morphOps(Mat &thresh);
void imagefilter(Mat &input, Mat &output_r,Mat &output_b);
void calibrate(Mat &frame);
int x_offset=7;
int z_offset=40;

int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;

Mat distCoeffs;

int iMin_tracking_ball_size = 15;
double fontScale = 2;
int thickness = 3;
String text ;

void ball_detect(){
     Mat edges;  //assign a memory to save the edge images
     Mat frame, frame2;  //assign a memory to save the images
     Mat hsv_frame_red_canny,hsv_frame_blue_canny;
       //create a message for ball positions

     if(buffer.size().width==320){ //if the size of the image is 320x240, then resized it to 640x480
         cv::resize(buffer, frame, cv::Size(640, 480));
     }
     else{
         frame = buffer;
     }
     if(buffer2.size().width==320){ //if the size of the image is 320x240, then resized it to 640x480
         cv::resize(buffer2, frame2, cv::Size(640, 480));
     }
     else{
         frame2 = buffer2;
     }
	calibrate(frame);
	calibrate(frame2);

  	vector<Vec4i> hierarchy_r;
    vector<Vec4i> hierarchy_b;
    vector<vector<Point> > contours_r;
    vector<vector<Point> > contours_b;

    imagefilter(frame,hsv_frame_red_canny,hsv_frame_blue_canny);

    findContours(hsv_frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

 	vector<vector<Point> > contours_r_poly( contours_r.size() );
    vector<vector<Point> > contours_b_poly( contours_b.size() );
	vector<Point2f>center_r( contours_r.size() );
    vector<Point2f>center_b( contours_b.size() );
    vector<float>radius_r( contours_r.size() );
    vector<float>radius_b( contours_b.size() );
   	for( size_t i = 0; i < contours_r.size(); i++ ){
            approxPolyDP( contours_r[i], contours_r_poly[i], 3, true );
            minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] );
        }
	for( size_t i = 0; i < contours_b.size(); i++ ){
            approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
            minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
        }
	core_msgs::ball_position msg;
	  //create a message for ball positions


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

	double radius = 0.10;
    ball_list.scale.x=radius; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
    ball_list.scale.y=radius;
    ball_list.scale.z=radius;

    vector<float> ball_b_x, ball_b_y, ball_b_z, ball_b_radius;
    int count_b=0;
 	for( size_t i = 0; i< contours_b.size(); i++ ){
    	if((radius_b[i] > iMin_tracking_ball_size) && (15<center_b[i].x)&&(center_b[i].x < 605)){ //for erasing errors using min radius and side of the screen
            vector<float> ball2;
			int cx = center_b[i].x;
			int cy = center_b[i].y;
			Point center(cx,cy);
			float distance = buffer2.at<ushort>(cy,cx);
 			ball2=pixel2point(center, distance);
 			cx =ball2[0]-x_offset;
 			cy =ball2[1];
 			distance = ball2[2];
            count_b++;
            float color_ball = 0;

            msg.img_x.push_back(cx);
            msg.img_y.push_back(cy);
            msg.img_z.push_back(distance);
            msg.color.push_back(color_ball);

            float x1, y1, x2, y2;
            x1 =center_b[i].x;
		    x2 =center_b[i+1].x;
            y1=center_b[i].y;
		    y2= center_b[i+1].y;
            float diff = sqrt(pow((x1-x2),2)+pow((y1-y2),2));
            float l = radius_b[i];

            Scalar color = Scalar( 255, 0, 0);
            drawContours( hsv_frame_blue_canny, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            text = "blue";
            putText(frame, text, center_b[i],2,1,Scalar(0,255,0),2);
            circle( frame, center_b[i], (int)radius_b[i], color, 2, 8, 0 );
            if (abs(l)>diff)i++;
            //cout << "blue"<<i<<"\t"<<"x="<<cx<<"\t"<<distance<<endl;
		}
    }
        vector<float> ball_r_x, ball_r_y, ball_r_z, ball_r_radius;
    int count_r=0;
 	for( size_t i = 0; i< contours_r.size(); i++ ){
    	if((radius_r[i] > iMin_tracking_ball_size) && (15<center_r[i].x)&&(center_r[i].x < 605)){ //for erasing errors using min radius and side of the screen
            vector<float> ball2;
			int cx = center_r[i].x;
			int cy = center_r[i].y;
			Point center(cx,cy);
			float distance = buffer2.at<ushort>(cy,cx);
 			ball2=pixel2point(center, distance);
 			cx =ball2[0]-x_offset;
 			cy =ball2[1];
 			distance = ball2[2];
            count_r++;
            float color_ball = 1;

            msg.img_x.push_back(cx);
            msg.img_y.push_back(cy);
            msg.img_z.push_back(distance);
            msg.color.push_back(color_ball);


            float x1, y1, x2, y2;
            x1 =center_r[i].x;
		    x2 =center_r[i+1].x;
            y1=center_r[i].y;
		    y2= center_r[i+1].y;
            float diff = sqrt(pow((x1-x2),2)+pow((y1-y2),2));
            float l = radius_r[i];

            Scalar color = Scalar( 0, 0, 255);
            drawContours( hsv_frame_red_canny, contours_r_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            text = "red";
            putText(frame, text, center_r[i],2,1,Scalar(0,255,0),2);
            circle( frame, center_r[i], (int)radius_r[i], color, 2, 8, 0 );
            //cout << "red"<<i<<"\t"<<"x="<<cx<<"\t"<<distance<<endl;
            if (abs(l)>diff)i++;


		}
    }
    msg.size = count_r + count_b;
    for ( int i = 0 ; i < count_b+count_r; i++){
    cout << i << "x  = " << msg.img_x[i] << "z =  "<<msg.img_z[i] << "color = " <<msg.color[i] <<endl;

}
cout << msg.color.size()<< endl;
     cv:: imshow("Object Detection_HSV_R",hsv_frame_red);
     cv::imshow("Object Detection_HSV",hsv_frame_blue);
     cv::imshow("view", frame);  //show the image with a window
     //cv::imshow("view2", frame2);  //show the image with a window
     cv::waitKey(1);
     pub.publish(msg);  //publish a message
     pub_markers.publish(ball_list);  //publish a marker message
msg.color.clear();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

   if(msg->height==480&&buffer.size().width==320){  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
	std::cout<<"resized"<<std::endl;
	cv::resize(buffer,buffer,cv::Size(640,480));
}
   else{
	//do nothing!
   }

   try
   {
     buffer = cv_bridge::toCvShare(msg, "bgr8")->image;  //transfer the image data into buffer
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
   ball_detect(); //proceed ball detection
}
void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
   //ROS_INFO("Depth image height = %d, width = %d",msg->height, msg->width);
   cv_bridge::CvImageConstPtr cv_ptrD;
   if(msg->height==480&&buffer2.size().width==320){  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
			std::cout<<"resized"<<std::endl;
			cv::resize(buffer2,buffer2,cv::Size(640,480));
	}
   try
   {

      cv_ptrD = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      buffer2 = cv_ptrD->image;
      if (buffer2.empty())
      {
         cerr << endl << "Failed to load image at: " << endl;
         return;
      }
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }

}

int main(int argc, char **argv)
{
  // Trackbars to set thresholds for HSV values : Blue ball
  // namedWindow("Object Detection_HSV_Blue1", WINDOW_NORMAL);
  // namedWindow("Object Detection_HSV_Red1", WINDOW_NORMAL);
  // moveWindow("Object Detection_HSV_Red1", 50,370);
  // moveWindow("Object Detection_HSV_Blue1", 50,370);
  // createTrackbar("Low H","Object Detection_HSV_Blue1", &low_h_b, 180, on_low_h_thresh_trackbar_blue_1);
  // createTrackbar("High H","Object Detection_HSV_Blue1", &high_h_b, 180, on_high_h_thresh_trackbar_blue_1);
  // createTrackbar("Low S","Object Detection_HSV_Blue1", &low_s_b, 255, on_low_s_thresh_trackbar_blue_1);
  // createTrackbar("High S","Object Detection_HSV_Blue1", &high_s_b, 255, on_high_s_thresh_trackbar_blue_1);
  // createTrackbar("Low V","Object Detection_HSV_Blue1", &low_v_b, 255, on_low_v_thresh_trackbar_blue_1);
  // createTrackbar("High V","Object Detection_HSV_Blue1", &high_v_b, 255, on_high_v_thresh_trackbar_blue_1);
  //
  //
  // createTrackbar("Low H","Object Detection_HSV_Red1", &low_h_r_1, 180, on_low_h_thresh_trackbar_red_1);
  // createTrackbar("High H","Object Detection_HSV_Red1", &high_h_r_1, 180, on_high_h_thresh_trackbar_red_1);
  // createTrackbar("Low H2","Object Detection_HSV_Red1", &low_h_r_2, 180, on_low_h2_thresh_trackbar_red_1);
  // createTrackbar("High H2","Object Detection_HSV_Red1", &high_h_r_2, 180, on_high_h2_thresh_trackbar_red_1);
  // createTrackbar("Low S","Object Detection_HSV_Red1", &low_s_r_1, 255, on_low_s_thresh_trackbar_red_1);
  // createTrackbar("High S","Object Detection_HSV_Red1", &high_s_r_1, 255, on_high_s_thresh_trackbar_red_1);
  // createTrackbar("Low V","Object Detection_HSV_Red1", &low_v_r_1, 255, on_low_v_thresh_trackbar_red_1);
  // createTrackbar("High V","Object Detection_HSV_Red1", &high_v_r_1, 255, on_high_v_thresh_trackbar_red_1);


   ros::init(argc, argv, "ball_detect_node"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
 image_transport::Subscriber sub2 = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthCallback);
   image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback); //create


   pub = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher
   pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);

   ros::spin(); //spin.
   return 0;
}

vector<float> pixel2point(Point center, int distance){vector<float> position;
    float x, y, u, v, Xc, Yc,Zc;
    x = center.x;//.x;// .at(0);
    y = center.y;//.y;//
    u = (x-intrinsic_data[2])/intrinsic_data[0];
    v = (y-intrinsic_data[5])/intrinsic_data[4];
    Zc = distance;
    Xc = u*Zc ;
    Yc = v*Zc ;
    Xc = roundf(Xc * 1000) / 1000;
    Yc = roundf(Yc * 1000) / 1000;
    //Zc = roundf(Zc * 1000) / 1000;
    distance = sqrt(pow(Zc,2)-pow(car_height,2)-pow(Xc,2));
    distance = distance-z_offset;
    position.push_back(Xc);
    position.push_back(Yc);
    position.push_back(distance);
    return position;
}

void morphOps(Mat &thresh){//create structuring element that will be used to "dilate" and "erode" image.: These are morphological operations, they process input image based on shape and generate an output image. Erosion and dilation remove noise, isolate individual elements, join disparate elements, , and find intensity bumps or holes.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);
    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
}

void calibrate(Mat &frame){
	Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs, output;
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);
    output = frame.clone();
	undistort(frame, output, intrinsic, distCoeffs);
	frame = output.clone();
}


void imagefilter(Mat &input, Mat &output_r,Mat &output_b){
	Mat hsv_frame, hsv_frame_red1,hsv_frame_red2, hsv_frame_red_blur, hsv_frame_blue_blur,hsv_frame_red_canny, hsv_frame_blue_canny;
	medianBlur(input, input, 3);
	cvtColor(input, hsv_frame, cv::COLOR_BGR2HSV);
	inRange(hsv_frame,Scalar(low_h_r_1,low_s_r_1,low_v_r_1),Scalar(high_h_r_1,high_s_r_1,high_v_r_1),hsv_frame_red1);
    inRange(hsv_frame,Scalar(low_h_r_2,low_s_r_1,low_v_r_1),Scalar(high_h_r_2,high_s_r_1,high_v_r_1),hsv_frame_red2);
    inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);
	addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);

 	morphOps(hsv_frame_red);
    morphOps(hsv_frame_blue);

	GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
    GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);
 	Canny(hsv_frame_red_blur, output_r, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
    Canny(hsv_frame_blue_blur, output_b, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);

}
// void on_low_h_thresh_trackbar_blue_1(int, void *){
// low_h_b = min(high_h_b-1, low_h_b);
// setTrackbarPos("Low H","Object Detection_HSV_Blue1", low_h_b);
// }
// void on_high_h_thresh_trackbar_blue_1(int, void *){
// high_h_b = max(high_h_b, low_h_b+1);
// setTrackbarPos("High H", "Object Detection_HSV_Blue1", high_h_b);
// }
// void on_low_s_thresh_trackbar_blue_1(int, void *){
// low_s_b = min(high_s_b-1, low_s_b);
// setTrackbarPos("Low S","Object Detection_HSV_Blue1", low_s_b);
// }
// void on_high_s_thresh_trackbar_blue_1(int, void *){
// high_s_b = max(high_s_b, low_s_b+1);
// setTrackbarPos("High S", "Object Detection_HSV_Blue1", high_s_b);
// }
// void on_low_v_thresh_trackbar_blue_1(int, void *){
// low_v_b= min(high_v_b-1, low_v_b);
// setTrackbarPos("Low V","Object Detection_HSV_Blue1", low_v_b);
// }
// void on_high_v_thresh_trackbar_blue_1(int, void *){
// high_v_b = max(high_v_b, low_v_b+1);
// setTrackbarPos("High V", "Object Detection_HSV_Blue1", high_v_b);
// }
//
//
// void on_low_h_thresh_trackbar_red_1(int, void *){
// low_h_r_1 = min(high_h_r_1-1, low_h_r_1);
// setTrackbarPos("Low H","Object Detection_HSV_Red1", low_h_r_1);
// }
// void on_high_h_thresh_trackbar_red_1(int, void *){
// high_h_r_1 = max(high_h_r_1, low_h_r_1+1);
// setTrackbarPos("High H", "Object Detection_HSV_Red1", high_h_r_1);
// }
// void on_low_h2_thresh_trackbar_red_1(int, void *){
// low_h_r_2 = min(high_h_r_2-1, low_h_r_2);
// setTrackbarPos("Low H2","Object Detection_HSV_Red1", low_h_r_2);
// }
// void on_high_h2_thresh_trackbar_red_1(int, void *){
// high_h_r_2 = max(high_h_r_2, low_h_r_2+1);
// setTrackbarPos("High H2", "Object Detection_HSV_Red1", high_h_r_2);
// }
// void on_low_s_thresh_trackbar_red_1(int, void *){
// low_s_r_1 = min(high_s_r_1-1, low_s_r_1);
// setTrackbarPos("Low S","Object Detection_HSV_Red1", low_s_r_1);
// }
// void on_high_s_thresh_trackbar_red_1(int, void *){
// high_s_r_1 = max(high_s_r_1, low_s_r_1+1);
// setTrackbarPos("High S", "Object Detection_HSV_Red1", high_s_r_1);
// }
// void on_low_v_thresh_trackbar_red_1(int, void *){
// low_v_r_1= min(high_v_r_1-1, low_v_r_1);
// setTrackbarPos("Low V","Object Detection_HSV_Red1", low_v_r_1);
// }
// void on_high_v_thresh_trackbar_red_1(int, void *){
// high_v_r_1 = max(high_v_r_1, low_v_r_1+1);
// setTrackbarPos("High V", "Object Detection_HSV_Red1", high_v_r_1);
// }

// Trackbar for image threshodling in HSV colorspace : Red2
