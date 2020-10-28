#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>

using namespace cv;
using namespace std;

Mat frame;

// Declaration of trackbar functions to set HSV colorspace's parameters
void on_low_h_thresh_trackbar_red(int, void *);
void on_high_h_thresh_trackbar_red(int, void *);
void on_low_s_thresh_trackbar_red(int, void *);
void on_high_s_thresh_trackbar_red(int, void *);
void on_low_v_thresh_trackbar_red(int, void *);
void on_high_v_thresh_trackbar_red(int, void *);
int low_h2_r=160, high_h2_r=180;
int low_h_r=0, low_s_r=160, low_v_r=58;
int high_h_r=10, high_s_r=255, high_v_r=255;

void on_low_h_thresh_trackbar_blue(int, void *);
void on_high_h_thresh_trackbar_blue(int, void *);
void on_low_s_thresh_trackbar_blue(int, void *);
void on_high_s_thresh_trackbar_blue(int, void *);
void on_low_v_thresh_trackbar_blue(int, void *);
void on_high_v_thresh_trackbar_blue(int, void *);
int low_h_b=100, low_s_b=148, low_v_b=73; //135
int high_h_b=120, high_s_b=255, high_v_b=255; //205

void on_low_h_thresh_trackbar_green(int, void *);
void on_high_h_thresh_trackbar_green(int, void *);
void on_low_s_thresh_trackbar_green(int, void *);
void on_high_s_thresh_trackbar_green(int, void *);
void on_low_v_thresh_trackbar_green(int, void *);
void on_high_v_thresh_trackbar_green(int, void *);
int low_h_g=45, low_s_g=150, low_v_g=35;
int high_h_g=90, high_s_g=255, high_v_g=255;

// Declaration of functions that changes data types
string intToString(int n);
string floatToString(float f);

// Declaration of functions that changes int data to String
void morphOps(Mat &thresh);

// Declaration of functions that calculates the ball position from pixel position
vector<float> pixel2point(Point center, int radius);

// Declaration of trackbars function that set canny edge's parameters
void on_canny_edge_trackbar_red(int, void *);
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;

void on_canny_edge_trackbar_blue(int, void *);
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;

void on_canny_edge_trackbar_green(int, void *);
int lowThreshold_g = 100;
int ratio_g = 3;
int kernel_size_g = 3;

// Initialization of variable for dimension of the target
float fball_radius = 0.073 ; // ball diameter, unit: meter

// Initialization of variable for camera calibration paramters
Mat distCoeffs;
// 640 x 480
float intrinsic_data[9] = {638.301619, 0.000000, 308.272626,
        0.000000, 634.514892, 232.583358,
        0.000000, 0.000000, 1.000000};

float distortion_data[5] = {0.034734, -0.154281, -0.003674, -0.001274, 0.000000};

// Initialization of variable for text drawing
double fontScale = 1;
int thickness = 1;
String text ;
int iMin_tracking_ball_size = 20;

// Mat buffer(240,320,CV_8UC1);
ros::Publisher pub;

void ball_detect(){
  Mat hsv_frame,
  hsv_frame_red, hsv_frame_blue, hsv_frame_green, hsv_frame_red2, hsv_frame_red1,
  hsv_frame_red_blur, hsv_frame_blue_blur, hsv_frame_green_blur,
  hsv_frame_red_canny, hsv_frame_blue_canny, hsv_frame_green_canny,
  result;

  Mat calibrated_frame;
  Mat intrinsic = Mat(3,3, CV_32FC1);
  Mat distCoeffs;
  intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  distCoeffs = Mat(1, 5, CV_32F, distortion_data);
  vector<Vec4i> hierarchy_r;
  vector<Vec4i> hierarchy_b;
  vector<Vec4i> hierarchy_g;
  vector<vector<Point> > contours_r;
  vector<vector<Point> > contours_b;
  vector<vector<Point> > contours_g;

     // if(buffer.size().width==320){ //if the size of the image is 320x240, then resized it to 640x480
         // cv::resize(buffer, frame, cv::Size(480, 640));
     // }
     // else{
         // frame = buffer;
     // }
	undistort(frame, calibrated_frame, intrinsic, distCoeffs);
  // calibrated_frame = calibrated_frame(cv::Rect(3, 0, 320, 480));
  result = calibrated_frame.clone();
	medianBlur(calibrated_frame, calibrated_frame, 3);
	cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

	// Detect the object based on RGB and HSV Range Values
  inRange(hsv_frame,Scalar(low_h_r,low_s_r,low_v_r),Scalar(high_h_r,high_s_r,high_v_r),hsv_frame_red1);
  inRange(hsv_frame,Scalar(low_h2_r,low_s_r,low_v_r),Scalar(high_h2_r,high_s_r,high_v_r),hsv_frame_red2);
  inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);
  // inRange(hsv_frame,Scalar(low_h_g,low_s_g,low_v_g),Scalar(high_h_g,high_s_g,high_v_g),hsv_frame_green);

  addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);
	morphOps(hsv_frame_red);
	morphOps(hsv_frame_blue);
	// morphOps(hsv_frame_green);

	GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
	GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);
	// GaussianBlur(hsv_frame_green, hsv_frame_green_blur, cv::Size(9, 9), 2, 2);

	Canny(hsv_frame_red_blur,hsv_frame_red_canny, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
	Canny(hsv_frame_blue_blur,hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);
	// Canny(hsv_frame_green_blur,hsv_frame_green_canny, lowThreshold_g, lowThreshold_g*ratio_g, kernel_size_g);

	findContours(hsv_frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
	findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
	// findContours(hsv_frame_green_canny, contours_g, hierarchy_g, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<vector<Point> > contours_r_poly( contours_r.size() );
	vector<vector<Point> > contours_b_poly( contours_b.size() );
	// vector<vector<Point> > contours_g_poly( contours_g.size() );

	vector<Point2f>center_r( contours_r.size() );
	vector<Point2f>center_b( contours_b.size() );
	// vector<Point2f>center_g( contours_g.size() );

	vector<float>radius_r( contours_r.size() );
	vector<float>radius_b( contours_b.size() );
	// vector<float>radius_g( contours_g.size() );



	for( size_t i = 0; i < contours_r.size(); i++ ){
		approxPolyDP( contours_r[i], contours_r_poly[i], 3, true );
		minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] );
	}
	for( size_t i = 0; i < contours_b.size(); i++ ){
		approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
		minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
	}
	// for( size_t i = 0; i < contours_g.size(); i++ ){
		// approxPolyDP( contours_g[i], contours_g_poly[i], 3, true );
		// minEnclosingCircle( contours_g_poly[i], center_g[i], radius_g[i] );
	// }

        float l, r1, r2;
        Point2f one, two;
        float x1, x2, y1, y2;

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

        contour_b = contours_b.size();
        for (size_t i=0; i<contour_b; i++){
            for (size_t j=0; j<contour_b; j++){
                r1=radius_b[i];
                r2=radius_b[j];
                if (r1>r2){
                    radius_b.erase(radius_b.begin()+j);
                    center_b.erase(center_b.begin()+j);
                    contours_b.erase(contours_b.begin()+j);
                    contour_b--;
                    i--;
                }
            }
        }

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

        contour_r = contours_r.size();
        for (size_t i=0; i<contour_r; i++){
            for (size_t j=0; j<contour_r; j++){
                r1=radius_r[i];
                r2=radius_r[j];
                if (r1>r2){
                    radius_r.erase(radius_r.begin()+j);
                    center_r.erase(center_r.begin()+j);
                    contours_r.erase(contours_r.begin()+j);
                    contour_r--;
                    i--;
                }
            }
        }

        int upper_limit = 479;
        int lower_limit = 1;
        int x_rect =  470;
        int y_rect = 280;

        size_t contour_b2 = contours_b.size();
        for (size_t i=0; i<contour_b2; i++){
           one=center_b[i];
           x1=one.x;
           y1=one.y;
           if (y1>upper_limit){
               radius_b.erase(radius_b.begin()+i);
               center_b.erase(center_b.begin()+i);
               contours_b.erase(contours_b.begin()+i);
               contour_b2--;
               i--;
           }
           if(y1<lower_limit){
               radius_b.erase(radius_b.begin()+i);
               center_b.erase(center_b.begin()+i);
               contours_b.erase(contours_b.begin()+i);
               contour_b2--;
               i--;
           }
           if((y1<y_rect)&&(x1>x_rect)){
             radius_b.erase(radius_b.begin()+i);
             center_b.erase(center_b.begin()+i);
             contours_b.erase(contours_b.begin()+i);
             contour_b2--;
             i--;
           }
        }

        size_t contour_r2 = contours_r.size();
        for (size_t i=0; i<contour_r2; i++){
           one=center_r[i];
           x1=one.x;
           y1=one.y;
           if (y1>upper_limit){
               radius_r.erase(radius_r.begin()+i);
               center_r.erase(center_r.begin()+i);
               contours_r.erase(contours_r.begin()+i);
               contour_r2--;
               i--;
           }
          if(y1<lower_limit){
               radius_r.erase(radius_r.begin()+i);
               center_r.erase(center_r.begin()+i);
               contours_r.erase(contours_r.begin()+i);
               contour_r2--;
               i--;
          }
          if((y1<y_rect)&&(x1>x_rect)){
            radius_r.erase(radius_r.begin()+i);
            center_r.erase(center_r.begin()+i);
            contours_r.erase(contours_r.begin()+i);
            contour_r2--;
            i--;
          }
        }
        core_msgs::ball_position msg;  //create a message for ball positions
        msg.size =contours_b.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
        msg.img_x.resize(contours_b.size());  //adjust the size of array
        msg.img_y.resize(contours_b.size());  //adjust the size of array

        msg.size2 = contours_r.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
        msg.img_x2.resize(contours_r.size());  //adjust the size of array
        msg.img_y2.resize(contours_r.size());  //adjust the size of array

  cout<<"blue? : "<<contours_b.size()<<endl;

	for( size_t i = 0; i< contours_r.size(); i++ ){
		if (radius_r[i] > iMin_tracking_ball_size){
			Scalar color = Scalar( 0, 0, 255);
			vector<float> ball_position_r;
			ball_position_r = pixel2point(center_r[i], radius_r[i]);
			drawContours( hsv_frame_red_canny, contours_r_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			float isx = ball_position_r[0];
			float isy = ball_position_r[1];
			float isz = ball_position_r[2];
            float pixels = 480-center_r[i].y;
      string sx = floatToString(isx);
			string sy = floatToString(isy);
			string sz = floatToString(isz);
      string center = floatToString(pixels);

                        msg.img_x2[i] = isx;
                        msg.img_y2[i] = (480-center_r[i].y);

			text = "R:" + center;
			putText(result, text, center_r[i],2,1,Scalar(0,255,0),2);
			circle( result, center_r[i], (int)radius_r[i], color, 2, 8, 0 );
		}
	}

	for( size_t i = 0; i< contours_b.size(); i++ ){
		if(radius_b[i] > iMin_tracking_ball_size){
			Scalar color = Scalar( 255, 0, 0);
			vector<float> ball_position_b;
			ball_position_b = pixel2point(center_b[i], radius_b[i]);
			drawContours( hsv_frame_blue_canny, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			float isx = ball_position_b[0];
			float isy = ball_position_b[1];
			float isz = ball_position_b[2];
      float pixels = 480-center_b[i].y;
			string sx = floatToString(isx);
			string sy = floatToString(isy);
			string sz = floatToString(isz);
      string center = floatToString(pixels);
                        msg.img_x[i] = isx;
                        msg.img_y[i] = (480-center_b[i].y);

			text = "B:" + center;
			putText(result, text, center_b[i],2,1,Scalar(0,255,0),2);
			circle( result, center_b[i], (int)radius_b[i], color, 2, 8, 0 );
		}
	}
	// for( size_t i = 0; i< contours_g.size(); i++ ){
	// 	if(radius_g[i] > iMin_tracking_ball_size){
	// 		Scalar color = Scalar( 0, 255, 0);
	// 		vector<float> ball_position_g;
	// 		ball_position_g = pixel2point(center_g[i], radius_g[i]);
	// 		drawContours( hsv_frame_green_canny, contours_g_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
	// 		float isx = ball_position_g[0];
	// 		float isy = ball_position_g[1];
	// 		float isz = ball_position_g[2];
  //           float pixels = -center_g[i].y;
	// 		string sx = floatToString(isx);
	// 		string sy = floatToString(isy);
  //     string sz = floatToString(isz);
  //     string center = floatToString(pixels);
  //                       msg.img_x3[i] = isx;
  //                       msg.img_y3[i] = isz;
  //
	// 		text = "G:" + center;
	// 		putText(result, text, center_g[i],2,1,Scalar(0,255,0),2);
	// 		circle( result, center_g[i], (int)radius_g[i], color, 2 , 8, 0 );
	// 	}
	// }
  cv::waitKey(1);
	imshow("Video Capture",calibrated_frame);
	imshow("Object Detection_HSV_Red",hsv_frame_red);
	imshow("Object Detection_HSV_Blue",hsv_frame_blue);
	imshow("Canny Edge for Red Ball", hsv_frame_red_canny);
	imshow("Canny Edge for Blue Ball", hsv_frame_blue_canny);
	imshow("Result", result);

	pub.publish(msg);
}

int main(int argc, char **argv)
{
	VideoCapture cap(4);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  namedWindow("Video Capture", WINDOW_NORMAL);
   namedWindow("Object Detection_HSV_Red", WINDOW_NORMAL);
   namedWindow("Object Detection_HSV_Blue", WINDOW_NORMAL);
   namedWindow("Canny Edge for Red Ball", WINDOW_NORMAL);
   namedWindow("Canny Edge for Blue Ball", WINDOW_NORMAL);
   namedWindow("Result", WINDOW_NORMAL);
   moveWindow("Video Capture", 50, 0);
   moveWindow("Object Detection_HSV_Red", 50,370);
   moveWindow("Object Detection_HSV_Blue",470,370);
   moveWindow("Canny Edge for Red Ball", 50,730);
   moveWindow("Canny Edge for Blue Ball", 470,730);
   moveWindow("Result", 470, 0);

    // Trackbars to set thresholds for HSV values : Red ball
   createTrackbar("Low H","Object Detection_HSV_Red", &low_h_r, 180, on_low_h_thresh_trackbar_red);
   createTrackbar("High H","Object Detection_HSV_Red", &high_h_r, 180, on_high_h_thresh_trackbar_red);
    // createTrackbar("Low H2","Object Detection_HSV_Red", &low_h2_r, 180, on_low_h2_thresh_trackbar_red);
    // createTrackbar("High H2","Object Detection_HSV_Red", &high_h2_r, 180, on_high_h2_thresh_trackbar_red);
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
    // Trackbar to set parameter for Canny Edge
    createTrackbar("Min Threshold:","Canny Edge for Red Ball", &lowThreshold_r, 100, on_canny_edge_trackbar_red);
    createTrackbar("Min Threshold:","Canny Edge for Blue Ball", &lowThreshold_b, 100, on_canny_edge_trackbar_blue);

   ros::init(argc, argv, "ball_detect_near_show_node"); //init ros nodd
   ros::NodeHandle nh; //create node handler

   pub = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher

   while (ros::ok()){
     cap>>frame;
     ball_detect();
     // ros::Duration(0.05).sleep();
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
vector<float> pixel2point(Point center, int radius){
    vector<float> position;
    float x, y, u, v, Xc, Yc, Zc;
    x = center.x;//.x;// .at(0);
    y = center.y;//.y;//
    u = (x-intrinsic_data[2])/intrinsic_data[0];
    v = (y-intrinsic_data[5])/intrinsic_data[4];
    Zc = (intrinsic_data[0]*fball_radius)/(2*(float)radius) ;
    Xc = u*Zc ;
    Yc = v*Zc ;
    Xc = roundf(Xc * 1000) / 1000;
    Yc = roundf(Yc * 1000) / 1000;
    Zc = roundf(Zc * 1000) / 1000;
    position.push_back(Xc);
    position.push_back(Yc);
    position.push_back(Zc);
    return position;
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

// Trackbar for image threshodling in HSV colorspace : Green
void on_low_h_thresh_trackbar_green(int, void *){
    low_h_g = min(high_h_g-1, low_h_g);
    setTrackbarPos("Low H","Object Detection_HSV_Green", low_h_g);
}
void on_high_h_thresh_trackbar_green(int, void *){
    high_h_g = max(high_h_g, low_h_g+1);
    setTrackbarPos("High H", "Object Detection_HSV_Blue", high_h_g);
}
void on_low_s_thresh_trackbar_green(int, void *){
    low_s_g = min(high_s_g-1, low_s_g);
    setTrackbarPos("Low S","Object Detection_HSV_Green", low_s_g);
}
void on_high_s_thresh_trackbar_green(int, void *){
    high_s_g = max(high_s_g, low_s_g+1);
    setTrackbarPos("High S", "Object Detection_HSV_Green", high_s_g);
}
void on_low_v_thresh_trackbar_green(int, void *){
    low_v_g= min(high_v_g-1, low_v_g);
    setTrackbarPos("Low V","Object Detection_HSV_Green", low_v_g);
}
void on_high_v_thresh_trackbar_green(int, void *){
    high_v_g = max(high_v_g, low_v_g+1);
    setTrackbarPos("High V", "Object Detection_HSV_Green", high_v_g);
}

// Trackbar for Canny edge algorithm
void on_canny_edge_trackbar_red(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Red Ball", lowThreshold_r);
}
void on_canny_edge_trackbar_blue(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Blue Ball", lowThreshold_b);
}
void on_canny_edge_trackbar_green(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Green Ball", lowThreshold_g);
}
