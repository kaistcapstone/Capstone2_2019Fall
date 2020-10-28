#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include "core_msgs/ball_bottom.h"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Int8.h"

#define INDEX_DEFAULT 3

using namespace std;
using namespace cv;

// Declaration of trackbar functions to set HSV colorspace's parameters: In this section, we declare the functions that are displayed at the end on the multiple trackbars. We declare two sets of trackbar functions; one for the red ball, and one for the blue ball, Void functions are used so that they do not return any value. int is used when integer values are desired.
int low_h2_r=155, high_h2_r=180;
int low_h_r=0, low_s_r=195, low_v_r=50;
int high_h_r=15, high_s_r=255, high_v_r=255;
void on_low_h_thresh_trackbar_red_1(int, void *);
void on_high_h_thresh_trackbar_red_1(int, void *);
void on_low_h2_thresh_trackbar_red_1(int, void *);
void on_high_h2_thresh_trackbar_red_1(int, void *);
void on_low_s_thresh_trackbar_red_1(int, void *);
void on_high_s_thresh_trackbar_red_1(int, void *);
void on_low_v_thresh_trackbar_red_1(int, void *);
void on_high_v_thresh_trackbar_red_1(int, void *);

int low_h_b=90, low_s_b=175, low_v_b=80;
int high_h_b=113, high_s_b=255, high_v_b=255;
void on_low_h_thresh_trackbar_blue_1(int, void *);
void on_high_h_thresh_trackbar_blue_1(int, void *);
void on_low_s_thresh_trackbar_blue_1(int, void *);
void on_high_s_thresh_trackbar_blue_1(int, void *);
void on_low_v_thresh_trackbar_blue_1(int, void *);
void on_high_v_thresh_trackbar_blue_1(int, void *);


// Declaration of functions that changes data types: Here, we declare functions that change the type: from integer to string, and from float to string respectively.
string intToString(int n);
string floatToString(float f);

// Declaration of functions that changes int data to String
// Later in the code, we create a structuring element that is used to "dilate" and "erode" image. Here we declare it to not return any value using void.
void morphOps(Mat &thresh);
int car_height=180;
int x_offset=0;
int z_offset=0;
// Declaration of functions that calculates the ball position from pixel position: Using the pixel positions from the image observed by the camera, this function calculates the position of the ball, which is extremely important in our course goal. The vector function stores series of elements with the same variable name, in float data type.
vector<float> pixel2point(Point center, int radius);
//set default value
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;
int lowThreshold_g = 100;
int ratio_g = 3;
int kernel_size_g = 3;

// Initialization of variable for dimension of the target: We set a float value for the radius of the desired targets, in this case the balls.
float fball_radius = 74 ; // meter: The unit which is used in the initialization.

// Initialization of variable for camera calibration paramters: Like we did in our second class, we have to calibrate our main camera, and obtain the intrinsic and distortion parameters in order to undistort the images seen.
Mat distCoeffs;
float intrinsic_data[9] = {652.568590, 0.000000, 311.288345,0.000000, 654.145768, 249.816006,0.000000, 0.000000, 1.000000};
float distortion_data[5] = {0.008400, -0.049278, 0.005863, -0.003016, 0.000000};


// Initialization of variable for text drawing: The text which we see at the results is defined here
double fontScale = 2;
int thickness = 3;
String text ;

int iMin_tracking_ball_size = 35; // This is the minimum tracking ball size, in pixels.

/////ROS publisher
ros::Publisher pub;
//ros::Publisher pub2;



void sigint_handler(int sig){
  exit(-1);
}

// Here, we start our main function.
int main(int argc, char **argv)
{ int idx = (argc == 1)? INDEX_DEFAULT : (atoi(argv[1]));
    signal(SIGINT, sigint_handler);

    ros::init(argc, argv, "ball_bottom_node"); //init ros nodd
    ros::NodeHandle nh; //create node handler
    pub = nh.advertise<core_msgs::ball_bottom>("/position2", 1); //setting publisher
    //pub2 = nh.advertise<std_msgs::Int8>("/ball_num", 100); //setting publisher

    core_msgs::ball_bottom msg;
//set frames using in image manipulation functions
    Mat frame, bgr_frame, hsv_frame, hsv_frame_red, hsv_frame_red1, hsv_frame_red2, hsv_frame_blue,hsv_frame_green, hsv_frame_red_blur, hsv_frame_blue_blur, hsv_frame_green_blur, hsv_frame_red_canny, hsv_frame_blue_canny,hsv_frame_green_canny, result;
    Mat calibrated_frame;
    Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs;
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);

//declare vectors using in findcontours function
    vector<Vec4i> hierarchy_r;
    vector<Vec4i> hierarchy_b;
    vector<Vec4i> hierarchy_g;

    vector<vector<Point> > contours_r;
    vector<vector<Point> > contours_b;
    vector<vector<Point> > contours_g;

    // Here, we start the video capturing function, with the argument being the camera being used. 0 indicates the default camera, and 1 indicates the additional camera. Also, we make the 6 windows which we see at the results.
    VideoCapture cap(idx);
	namedWindow("Result", WINDOW_NORMAL);
	moveWindow("Result", 470, 0);


    while((char)waitKey(1)!='q'){
    cap>>frame;
    if(frame.empty())
        break;
        namedWindow("Object Detection_HSV_Blue1", WINDOW_NORMAL);
        namedWindow("Object Detection_HSV_Red1", WINDOW_NORMAL);
        moveWindow("Object Detection_HSV_Red1", 50,370);
        moveWindow("Object Detection_HSV_Blue1", 50,370);
        createTrackbar("Low H","Object Detection_HSV_Blue1", &low_h_b, 180, on_low_h_thresh_trackbar_blue_1);
        createTrackbar("High H","Object Detection_HSV_Blue1", &high_h_b, 180, on_high_h_thresh_trackbar_blue_1);
        createTrackbar("Low S","Object Detection_HSV_Blue1", &low_s_b, 255, on_low_s_thresh_trackbar_blue_1);
        createTrackbar("High S","Object Detection_HSV_Blue1", &high_s_b, 255, on_high_s_thresh_trackbar_blue_1);
        createTrackbar("Low V","Object Detection_HSV_Blue1", &low_v_b, 255, on_low_v_thresh_trackbar_blue_1);
        createTrackbar("High V","Object Detection_HSV_Blue1", &high_v_b, 255, on_high_v_thresh_trackbar_blue_1);


        createTrackbar("Low H","Object Detection_HSV_Red1", &low_h_r, 180, on_low_h_thresh_trackbar_red_1);
        createTrackbar("High H","Object Detection_HSV_Red1", &high_h_r, 180, on_high_h_thresh_trackbar_red_1);
        createTrackbar("Low H2","Object Detection_HSV_Red1", &low_h2_r, 180, on_low_h2_thresh_trackbar_red_1);
        createTrackbar("High H2","Object Detection_HSV_Red1", &high_h2_r, 180, on_high_h2_thresh_trackbar_red_1);
        createTrackbar("Low S","Object Detection_HSV_Red1", &low_s_r, 255, on_low_s_thresh_trackbar_red_1);
        createTrackbar("High S","Object Detection_HSV_Red1", &high_s_r, 255, on_high_s_thresh_trackbar_red_1);
        createTrackbar("Low V","Object Detection_HSV_Red1", &low_v_r, 255, on_low_v_thresh_trackbar_red_1);
        createTrackbar("High V","Object Detection_HSV_Red1", &high_v_r, 255, on_high_v_thresh_trackbar_red_1);

    undistort(frame, calibrated_frame, intrinsic, distCoeffs);//Using the intrinsic and distortion data obtained from the camera calibration, we undistort the viewed image.

    result = calibrated_frame.clone();

    medianBlur(calibrated_frame, calibrated_frame, 3);// Median blur function.

    cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);// Color is converted from BGR color space to HSV color space.

    // Detect the object based on RGB and HSV Range Values: In the following lines, the camera uses the BGR to HSV converted colors to detect the different colored objects.

    inRange(hsv_frame,Scalar(low_h_r,low_s_r,low_v_r),Scalar(high_h_r,high_s_r,high_v_r),hsv_frame_red1);

    inRange(hsv_frame,Scalar(low_h2_r,low_s_r,low_v_r),Scalar(high_h2_r,high_s_r,high_v_r),hsv_frame_red2);

    inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);


    addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);

    morphOps(hsv_frame_red);
    morphOps(hsv_frame_blue);

    // Gaussian blur function.
    GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
    GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);
    //Canny edge function.
    Canny(hsv_frame_red_blur, hsv_frame_red_canny, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
    Canny(hsv_frame_blue_blur, hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);

    //Find contour function.
    findContours(hsv_frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

    //resize to number of detected balls
    vector<vector<Point> > contours_r_poly( contours_r.size() );
    vector<vector<Point> > contours_b_poly( contours_b.size() );

    vector<Point2f>center_r( contours_r.size());
    vector<Point2f>center_b( contours_b.size() );

    vector<float>radius_r( contours_r.size() );
    vector<float>radius_b( contours_b.size() );

//Minenclosing circle function.
        for( size_t i = 0; i < contours_b.size(); i++ ){
            approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
            minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
        }
        for( size_t i = 0; i < contours_r.size(); i++ ){
            approxPolyDP( contours_r[i], contours_r_poly[i], 3, true );
            minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] );
        }

int count_r =0;
int count_b =0;

vector<float> ball_r_x, ball_r_y, ball_r_z, ball_r_radius;
vector<float> ball_b_x, ball_b_y, ball_b_z, ball_b_radius;
msg.img_x.clear();
msg.img_y.clear();
msg.img_z.clear();

//draw circles in screen and processing
    for( size_t i = 0; i< contours_b.size(); i++ ){
      if((radius_b[i] > iMin_tracking_ball_size)&& (15<center_b[i].x)&&(center_b[i].x < 605)){ //for erasing errors using min radius and side of the screen
            vector<float> ball_position_b;
            ball_position_b = pixel2point(center_b[i], radius_b[i]);
            int color_ball = 2;
		    msg.img_x.push_back(ball_position_b[0]);
            msg.img_y.push_back(ball_position_b[1]);
            msg.img_z.push_back(ball_position_b[2]);
            msg.color.push_back(color_ball);
		    count_b++;
		//같은 공 여러개로 보이는 것
            int x1, y1, x2, y2;
            x1 =center_b[i].x;
		    x2 =center_b[i+1].x;
            y1=center_b[i].y;
		    y2= center_b[i+1].y;
            float diff = sqrt(pow((x1-x2),2)+pow((y1-y2),2));
            float l = radius_b[i];
//cout << "y1 "<<y1 <<endl;
                Scalar color = Scalar( 255, 0, 0);
                drawContours( hsv_frame_blue_canny, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                text = "blue";
                putText(result, text, center_b[i],2,1,Scalar(0,255,0),2);
                circle( result, center_b[i], (int)radius_b[i], color, 2, 8, 0 );
		              //cout<<"blue"<<i<<"\t"<<"x="<<ball_position_b[0]<<"\t"<<"z="<<ball_position_b[2]<<endl;
                      //cout<<"msgcolor"<<msg.color[i]<<endl;

                if (abs(l)>diff)i++;
		}
        }


for( size_t i = 0; i< contours_r.size(); i++ ){
    if((radius_r[i] > iMin_tracking_ball_size)&& (15<center_r[i].x)&&(center_r[i].x < 605)){
          vector<float> ball_position_r;
          ball_position_r = pixel2point(center_r[i], radius_r[i]);
            int color_ball_r = 1;
            msg.img_x.push_back(ball_position_r[0]);
            msg.img_y.push_back(ball_position_r[1]);
            msg.img_z.push_back(ball_position_r[2]);
            msg.color.push_back(color_ball_r);
            float x1, y1, x2, y2;
            x1 =center_r[i].x;
            x2 =center_r[i+1].x;
            y1=center_r[i].y;
            y2= center_r[i+1].y;
            float diff = sqrt(pow((x1-x2),2)+pow((y1-y2),2));
            float l = radius_r[i];
              count_r++;
                Scalar color = Scalar( 0, 0, 255);
                drawContours( hsv_frame_red_canny, contours_r_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                text = "red";

                putText(result, text, center_r[i],2,1,Scalar(0,255,0),2);
                circle( result, center_r[i], (int)radius_r[i], color, 2, 8, 0 );
//cout<<"red"<<i<<"\t"<<"x="<<ball_position_r[0]<<"\t"<<"z="<<ball_position_r[2]<<endl;
//cout<<"msgcolor"<<msg.color[i+count_b]<<endl;

                if (abs(l)>diff)i++;
              }}

//msg에 계산된 값들 넣어줌
msg.size = count_b+count_r;

//publish
for ( int i = 0 ; i < count_b+count_r; i++){
    cout << i << "x  = " << msg.img_x[i] << "z =  "<<msg.img_z[i] << "color = " <<msg.color[i] <<endl;

}
cout << msg.color.size()<< endl;
    //std_msgs::Int8 num;
    //num.data = 1;
    //pub2.publish(num);
    pub.publish(msg);
    //cout << num.data<<endl;
    // Show the frames: Here, the 6 final widnows or frames are displayed for the user to see.

    cv:: imshow("Object Detection_HSV_R",hsv_frame_red);
    cv::imshow("Object Detection_HSV",hsv_frame_blue);
    cv::imshow("view", frame);
    imshow("Result", result);
msg.color.clear();


    }
    ros::spin();
    return 0;
}
//chage format int to string
string intToString(int n){stringstream s;
    s << n;
    return s.str();
}
//change format float to string
string floatToString(float f){ostringstream buffer;
    buffer << f;
    return buffer.str();
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
//calculate real distance using calibration.

vector<float> pixel2point(Point center, int radius){vector<float> position;
    float x, y, u, v, Xc, Yc, Zc,Pc;
    x = center.x;//.x;// .at(0);
    y = center.y;//.y;//
    u = (x-intrinsic_data[2])/intrinsic_data[0];
    v = (y-intrinsic_data[5])/intrinsic_data[4];

    //Zc = (intrinsic_data[0]*fball_radius)/(2*(float)radius) ;
Zc = 0.0018*pow(y,2)-2.2323*y+767.3;
	Xc = u*Zc ;
Yc = v*Zc ;
Zc=Zc-z_offset;

    //Xc = r;
    //Pc = sqrt(pow(Zc,2)-pow(car_height,2)-pow(Xc,2));
    //Pc = Pc - z_offset;
    //Yc = roundf(Yc * 1000) / 1000;
    //Zc = roundf(Zc * 1000) / 1000;
    position.push_back(Xc);
    position.push_back(Yc);
    position.push_back(Zc);
    return position;
}

void on_low_h_thresh_trackbar_blue_1(int, void *){
low_h_b = min(high_h_b-1, low_h_b);
setTrackbarPos("Low H","Object Detection_HSV_Blue1", low_h_b);
}
void on_high_h_thresh_trackbar_blue_1(int, void *){
high_h_b = max(high_h_b, low_h_b+1);
setTrackbarPos("High H", "Object Detection_HSV_Blue1", high_h_b);
}
void on_low_s_thresh_trackbar_blue_1(int, void *){
low_s_b = min(high_s_b-1, low_s_b);
setTrackbarPos("Low S","Object Detection_HSV_Blue1", low_s_b);
}
void on_high_s_thresh_trackbar_blue_1(int, void *){
high_s_b = max(high_s_b, low_s_b+1);
setTrackbarPos("High S", "Object Detection_HSV_Blue1", high_s_b);
}
void on_low_v_thresh_trackbar_blue_1(int, void *){
low_v_b= min(high_v_b-1, low_v_b);
setTrackbarPos("Low V","Object Detection_HSV_Blue1", low_v_b);
}
void on_high_v_thresh_trackbar_blue_1(int, void *){
high_v_b = max(high_v_b, low_v_b+1);
setTrackbarPos("High V", "Object Detection_HSV_Blue1", high_v_b);
}


void on_low_h_thresh_trackbar_red_1(int, void *){
low_h_r = min(high_h_r-1, low_h_r);
setTrackbarPos("Low H","Object Detection_HSV_Red1", low_h_r);
}
void on_high_h_thresh_trackbar_red_1(int, void *){
high_h_r = max(high_h_r, low_h_r+1);
setTrackbarPos("High H", "Object Detection_HSV_Red1", high_h_r);
}
void on_low_h2_thresh_trackbar_red_1(int, void *){
low_h2_r = min(high_h2_r-1, low_h2_r);
setTrackbarPos("Low H2","Object Detection_HSV_Red1", low_h2_r);
}
void on_high_h2_thresh_trackbar_red_1(int, void *){
high_h2_r = max(high_h2_r, low_h2_r+1);
setTrackbarPos("High H2", "Object Detection_HSV_Red1", high_h2_r);
}
void on_low_s_thresh_trackbar_red_1(int, void *){
low_s_r = min(high_s_r-1, low_s_r);
setTrackbarPos("Low S","Object Detection_HSV_Red1", low_s_r);
}
void on_high_s_thresh_trackbar_red_1(int, void *){
high_s_r = max(high_s_r, low_s_r+1);
setTrackbarPos("High S", "Object Detection_HSV_Red1", high_s_r);
}
void on_low_v_thresh_trackbar_red_1(int, void *){
low_v_r= min(high_v_r-1, low_v_r);
setTrackbarPos("Low V","Object Detection_HSV_Red1", low_v_r);
}
void on_high_v_thresh_trackbar_red_1(int, void *){
high_v_r = max(high_v_r, low_v_r+1);
setTrackbarPos("High V", "Object Detection_HSV_Red1", high_v_r);
}

// Trackbar for image threshodling in HSV colorspace : Red2
