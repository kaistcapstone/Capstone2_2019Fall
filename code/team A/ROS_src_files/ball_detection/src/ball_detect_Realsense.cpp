#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <librealsense2/rs.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>

using namespace std;

using namespace cv;

// Declaration of trackbar functions to set HSV colorspace's parameters
void on_low_h_thresh_trackbar_red(int, void *);
void on_high_h_thresh_trackbar_red(int, void *);
void on_low_h2_thresh_trackbar_red(int, void *);
void on_high_h2_thresh_trackbar_red(int, void *);
void on_low_s_thresh_trackbar_red(int, void *);
void on_high_s_thresh_trackbar_red(int, void *);
void on_low_v_thresh_trackbar_red(int, void *);
void on_high_v_thresh_trackbar_red(int, void *);
int low_h2_r=160, high_h2_r=180;
int low_h_r=0, low_s_r=104, low_v_r=100; //0, 142, 165(132)
int high_h_r=10, high_s_r=255, high_v_r=255;

void on_low_h_thresh_trackbar_blue(int, void *);
void on_high_h_thresh_trackbar_blue(int, void *);
void on_low_s_thresh_trackbar_blue(int, void *);
void on_high_s_thresh_trackbar_blue(int, void *);
void on_low_v_thresh_trackbar_blue(int, void *);
void on_high_v_thresh_trackbar_blue(int, void *);
int low_h_b=100, low_s_b=183, low_v_b=130;
int high_h_b=120, high_s_b=255, high_v_b=255;

// Declaration of functions that changes data types
string intToString(int n);
string floatToString(float f);

// Declaration of functions that changes int data to String
void morphOps(Mat &thresh);

// Declaration of functions that calculates the ball position from pixel position
vector<float> pixel2point(Point center, int radius);
float dist_correct(float f);

// Declaration of trackbars function that set canny edge's parameters
void on_canny_edge_trackbar_red(int, void *);
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;

void on_canny_edge_trackbar_blue(int, void *);
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;

double fontScale = 1;
int thickness = 1;
String text ;
int iMin_tracking_ball_size = 10;

ros::Publisher pub;
rs2::colorizer color_map;
rs2::pipeline realsense_pipe;
rs2::config cfg;


void ball_detect()
{
    Mat hsv_frame,
    hsv_frame_red, hsv_frame_red1, hsv_frame_red2, hsv_frame_blue,
    hsv_frame_red_blur, hsv_frame_blue_blur,
    hsv_frame_red_canny, hsv_frame_blue_canny, result;
    vector<Vec4i> hierarchy_r;
    vector<Vec4i> hierarchy_b;
    vector<vector<Point> > contours_r;
    vector<vector<Point> > contours_b;

    rs2::frameset data = realsense_pipe.wait_for_frames();
    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(data);
    rs2::video_frame colored = aligned_frames.first(RS2_STREAM_COLOR);
    rs2::depth_frame depth = aligned_frames.get_depth_frame();

    rs2_intrinsics intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    const int w = depth.as<rs2::video_frame>().get_width();
    const int h = depth.as<rs2::video_frame>().get_height();

    Mat image(Size(w, h), CV_8UC3, (void*)depth.apply_filter(color_map).get_data(), Mat::AUTO_STEP);
    Mat frame(Size(w, h), CV_8UC3, (void*)colored.get_data());
    
    if(frame.empty()){
      cout<<"empty frame"<<endl;
      return;

    }
    cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    result = frame.clone();
    medianBlur(frame, frame, 3);
    cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

    inRange(hsv_frame, Scalar(low_h_r,low_s_r,low_v_r), Scalar(high_h_r,high_s_r,high_v_r), hsv_frame_red1);
    inRange(hsv_frame, Scalar(low_h2_r,low_s_r,low_v_r), Scalar(high_h2_r,high_s_r,high_v_r), hsv_frame_red2);
    inRange(hsv_frame, Scalar(low_h_b,low_s_b,low_v_b), Scalar(high_h_b,high_s_b,high_v_b), hsv_frame_blue);
    addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);

    morphOps(hsv_frame_red);
    morphOps(hsv_frame_blue);
    GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
    GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);

    Canny(hsv_frame_red_blur,hsv_frame_red_canny, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
    Canny(hsv_frame_blue_blur,hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);

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
    float l, r1, r2;
    Point2f one, two;
    float x1, x2, y1, y2;

    size_t contour_r=contours_r.size();
    for (size_t i=0; i< contour_r; i++){
        if (radius_r[i] > iMin_tracking_ball_size){
            for (size_t j=0; j<contour_r; j++){
                for (size_t k=0; k<contour_r; k++){
                    one=center_r[j];
                    two=center_r[k];
                    r1=radius_r[j];
                    r2=radius_r[k];

                    x1=one.x;
                    y1=one.y;
                    x2=two.x;
                    y2=two.y;
                    l=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));

                    if (r1+r2>l){
                        if (r1>r2){
                            radius_r.erase(radius_r.begin()+k);
                            center_r.erase(center_r.begin()+k);
                            contours_r.erase(contours_r.begin()+k);
                            contour_r--;
                            j--;
                        }
                    }
                }
            }
        }
    }

    size_t contour_b=contours_b.size();
    for (size_t i=0; i< contour_b; i++){
        if (radius_b[i] > iMin_tracking_ball_size){
            for (size_t j=0; j<contour_b; j++){
                for (size_t k=0; k<contour_b; k++){
                    one=center_b[j];
                    two=center_b[k];
                    r1=radius_b[j];
                    r2=radius_b[k];

                    x1=one.x;
                    y1=one.y;
                    x2=two.x;
                    y2=two.y;
                    l=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));

                    if (r1+r2>l){
                        if (r1>r2){
                            radius_b.erase(radius_b.begin()+k);
                            center_b.erase(center_b.begin()+k);
                            contours_b.erase(contours_b.begin()+k);
                            contour_b--;
                            j--;
                        }
                    }
                }
            }
        }
    }

    core_msgs::ball_position msg;  //create a message for ball positions
    msg.size =contours_b.size()+contours_r.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
    msg.img_x.resize(msg.size);  //adjust the size of array
    msg.img_y.resize(msg.size);  //adjust the size of array
    msg.size2 = 0;

    for( size_t i = 0; i< contours_r.size(); i++ ){
        if(radius_r[i] > iMin_tracking_ball_size){
            Scalar color = Scalar( 255, 0, 0);
            drawContours( hsv_frame_red_canny, contours_r_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            string px = floatToString(roundf(center_r[i].x * 10) / 10);
            string py = floatToString(roundf(center_r[i].y * 10) / 10);
            circle( result, center_r[i], (int)radius_r[i], color, 2, 8, 0 );
            float x, y;
            float z = depth.get_distance((int)center_r[i].x,(int)center_r[i].y);
            x = z * (center_r[i].x - intr.ppx) / intr.fx;
            y = z * (center_r[i].y - intr.ppy) / intr.fy;
            msg.img_x[i] = x;
            msg.img_y[i] = z;
            cout<<"red xyz"<<endl;
        }
    }

    for( size_t i = 0; i< contours_b.size(); i++ ){
        if(radius_b[i] > iMin_tracking_ball_size){
            Scalar color = Scalar( 255, 0, 0);
            drawContours( hsv_frame_blue_canny, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            string px = floatToString(roundf(center_b[i].x * 10) / 10);
            string py = floatToString(roundf(center_b[i].y * 10) / 10);
            circle( result, center_b[i], (int)radius_b[i], color, 2, 8, 0 );
            float x, y;
            float z = depth.get_distance((int)center_b[i].x,(int)center_b[i].y);
            x = z * (center_b[i].x - intr.ppx) / intr.fx;
            y = z * (center_b[i].y - intr.ppy) / intr.fy;
            msg.img_x[i+contours_r.size()] = x;
            msg.img_y[i+contours_r.size()] = z;
            cout<<"blue xyz"<<endl;
        }
    }
    for(size_t i = 0; i<msg.size; i++){
        if(msg.img_y[i]==0){
           for(size_t j=i; j<msg.size; j++){
               if(j==msg.size){
                 msg.img_y[j]=100;
                 msg.img_x[j]=100;
               }
               else{
               msg.img_y[j] = msg.img_y[j+1];
               msg.img_x[j] = msg.img_x[j+1];
               }
           }
           msg.size--;
           i--;
        }
    }
    msg.img_x.resize(msg.size);  //adjust the size of array
    msg.img_y.resize(msg.size);  //adjust the size of array

    cv::waitKey(1);
    imshow("Video Capture", frame);
    imshow("depth image", image);
    imshow("Object Detection_HSV_Red",hsv_frame_red);
    imshow("Object Detection_HSV_Blue",hsv_frame_blue);
    imshow("Result", result);
    pub.publish(msg);
}

int main(int argc, char **argv)
{

    namedWindow("Video Capture", WINDOW_NORMAL);
    namedWindow("Object Detection_HSV_Red", WINDOW_NORMAL);
    namedWindow("Object Detection_HSV_Blue", WINDOW_NORMAL);
    namedWindow("depth image", WINDOW_NORMAL);
    namedWindow("Result", WINDOW_NORMAL);

    // Trackbars to set thresholds for HSV values : Red ball
    createTrackbar("Low H","Object Detection_HSV_Red", &low_h_r, 180, on_low_h_thresh_trackbar_red);
    createTrackbar("High H","Object Detection_HSV_Red", &high_h_r, 180, on_high_h_thresh_trackbar_red);
    createTrackbar("Low H2","Object Detection_HSV_Red", &low_h2_r, 180, on_low_h2_thresh_trackbar_red);
    createTrackbar("High H2","Object Detection_HSV_Red", &high_h2_r, 180, on_high_h2_thresh_trackbar_red);
    createTrackbar("Low S","Object Detection_HSV_Red", &low_s_r, 255, on_low_s_thresh_trackbar_red);
    createTrackbar("High S","Object Detection_HSV_Red", &high_s_r, 255, on_high_s_thresh_trackbar_red);
    createTrackbar("Low V","Object Detection_HSV_Red", &low_v_r, 255, on_low_v_thresh_trackbar_red);
    createTrackbar("High V","Object Detection_HSV_Red", &high_v_r, 255, on_high_v_thresh_trackbar_red);
    // Trackbars to set thresholds for HSV values : Blue ball
    createTrackbar("Low H","Object Detection_HSV_Blue", &low_h_b, 180, on_low_h_thresh_trackbar_blue);
    createTrackbar("High H","Object Detection_HSV_Blue", &high_h_b, 180, on_high_h_thresh_trackbar_blue);
    createTrackbar("Low S","Object Detection_HSV_Blue", &low_s_b, 255, on_low_s_thresh_trackbar_blue);
    createTrackbar("High S","Object Detection_HSV_Blue", &high_s_b, 255, on_high_s_thresh_trackbar_blue);
    createTrackbar("Low V","Object Detection_HSV_Blue", &low_v_b, 255, on_low_v_thresh_trackbar_blue);
    createTrackbar("High V","Object Detection_HSV_Blue", &high_v_b, 255, on_high_v_thresh_trackbar_blue);

   ros::init(argc, argv, "ball_detect_realsense_node"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   pub = nh.advertise<core_msgs::ball_position>("/position123", 1000); //setting publisher
   cfg.enable_stream(RS2_STREAM_COLOR, 640, 360);
   cfg.enable_stream(RS2_STREAM_DEPTH, 640, 360);
   // realsense_pipe.start(cfg);
   realsense_pipe.start();
   while (ros::ok()){
     ball_detect();
   }
   return 0;
}

string intToString(int n){
    stringstream s;
    s << n;
    return s.str();
}
string floatToString(float f){
    ostringstream buffer;
    buffer << f;
    return buffer.str();
}
void morphOps(Mat &thresh){
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
    erode(thresh,thresh,erodeElement); //remove blur noise, shrinked ball size
    erode(thresh,thresh,erodeElement);
    dilate(thresh,thresh,dilateElement); //original ball size
    dilate(thresh,thresh,dilateElement);
}

// Trackbar for image threshodling in HSV colorspace : Red
void on_low_h_thresh_trackbar_red(int, void *){
    low_h_r = min(high_h_r-1, low_h_r);
    setTrackbarPos("Low H","Object Detection_HSV_Red", low_h_r);
}
void on_high_h_thresh_trackbar_red(int, void *){
    high_h_r = max(high_h_r, low_h_r+1);
    setTrackbarPos("High H", "Object Detection_HSV_Red", high_h_r);
}
void on_low_h2_thresh_trackbar_red(int, void *){
    low_h2_r = min(high_h2_r-1, low_h2_r);
    setTrackbarPos("Low H2","Object Detection_HSV_Red", low_h2_r);
}
void on_high_h2_thresh_trackbar_red(int, void *){
    high_h_r = max(high_h2_r, low_h2_r+1);
    setTrackbarPos("High H", "Object Detection_HSV_Red", high_h2_r);
}
void on_low_s_thresh_trackbar_red(int, void *){
    low_s_r = min(high_s_r-1, low_s_r);
    setTrackbarPos("Low S","Object Detection_HSV_Red", low_s_r);
}
void on_high_s_thresh_trackbar_red(int, void *){
    high_s_r = max(high_s_r, low_s_r+1);
    setTrackbarPos("High S", "Object Detection_HSV_Red", high_s_r);
}
void on_low_v_thresh_trackbar_red(int, void *){
    low_v_r= min(high_v_r-1, low_v_r);
    setTrackbarPos("Low V","Object Detection_HSV_Red", low_v_r);
}
void on_high_v_thresh_trackbar_red(int, void *){
    high_v_r = max(high_v_r, low_v_r+1);
    setTrackbarPos("High V", "Object Detection_HSV_Red", high_v_r);
}

// Trackbar for image threshodling in HSV colorspace : Blue
void on_low_h_thresh_trackbar_blue(int, void *){
    low_h_b = min(high_h_b-1, low_h_b);
    setTrackbarPos("Low H","Object Detection_HSV_Blue", low_h_b);
}
void on_high_h_thresh_trackbar_blue(int, void *){
    high_h_b = max(high_h_b, low_h_b+1);
    setTrackbarPos("High H", "Object Detection_HSV_Blue", high_h_b);
}
void on_low_s_thresh_trackbar_blue(int, void *){
    low_s_b = min(high_s_b-1, low_s_b);
    setTrackbarPos("Low S","Object Detection_HSV_Blue", low_s_b);
}
void on_high_s_thresh_trackbar_blue(int, void *){
    high_s_b = max(high_s_b, low_s_b+1);
    setTrackbarPos("High S", "Object Detection_HSV_Blue", high_s_b);
}
void on_low_v_thresh_trackbar_blue(int, void *){
    low_v_b= min(high_v_b-1, low_v_b);
    setTrackbarPos("Low V","Object Detection_HSV_Blue", low_v_b);
}
void on_high_v_thresh_trackbar_blue(int, void *){
    high_v_b = max(high_v_b, low_v_b+1);
    setTrackbarPos("High V", "Object Detection_HSV_Blue", high_v_b);
}

// Trackbar for Canny edge algorithm
void on_canny_edge_trackbar_red(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Red Ball", lowThreshold_r);
}
void on_canny_edge_trackbar_blue(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Blue Ball", lowThreshold_b);
}
