// 2019.05.17. 21:30 by HS

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <stdio.h>
#include <std_msgs/Int32.h>

#define PI 3.14159265

using namespace std;
using namespace cv;

// demo place past
// int low_h2_r=169, high_h2_r=180;
// int low_h_r=0, low_s_r=90, low_v_r=112;
// int high_h_r=8, high_s_r=255, high_v_r=255;
// int low_h_b=100, low_s_b=126, low_v_b=85;
// int high_h_b=121, high_s_b=255, high_v_b=255;
// int low_h_g=65, low_s_g=80, low_v_g=50;
// int high_h_g=95, high_s_g=255, high_v_g=255;

// demo place
// int low_h2_r=169, high_h2_r=180;
// int low_h_r=0, low_s_r=136, low_v_r=97;
// int high_h_r=8, high_s_r=255, high_v_r=255;
int low_h2_r=169, high_h2_r=180;
int low_h_r=0, low_s_r=134, low_v_r=106;
int high_h_r=8, high_s_r=255, high_v_r=255;
int low_h_b=100, low_s_b=126, low_v_b=100;
int high_h_b=121, high_s_b=255, high_v_b=255;
//int low_h_g=52, low_s_g=93, low_v_g=35;
//int high_h_g=96, high_s_g=255, high_v_g=126;
// int low_h_g=43, low_s_g=68, low_v_g=62;
// int high_h_g=96, high_s_g=255, high_v_g=146;
int low_h_g=43, low_s_g=97, low_v_g=66;
int high_h_g=96, high_s_g=255, high_v_g=146;
// int low_h_g=43, low_s_g=97, low_v_g=20;
// int high_h_g=96, high_s_g=255, high_v_g=126;

// Initialization of variable for camera calibration paramters

//float intrinsic_data[9] = {1390.429077, 0, 991.590027, 0, 1390.645996, 544.835266, 0, 0, 1};
//float intrinsic_data[9] = {926.952718, 0, 661.060018, 0, 927.097331, 363.223511, 0, 0, 1};
float intrinsic_data[9] = {925.0222167, 0, 646.3529867, 0, 924.784912, 355.337158, 0, 0, 1};

float distortion_data[5] = {0, 0, 0, 0, 0};

// Initialization of variable for text drawing
String text;
int iMin_tracking_ball_size = 5;
float fball_radius = 0.072 ; // Initialization of variable for dimension of the target (meter)
float theta = 18;
int radcon=0;

/*
int low_h2_r=169, high_h2_r=180;
int low_h_r=0, low_s_r=90, low_v_r=112;
int high_h_r=8, high_s_r=255, high_v_r=255;
int low_h_b=100, low_s_b=126, low_v_b=60;
int high_h_b=121, high_s_b=255, high_v_b=255;
int low_h_g=55, low_s_g=80, low_v_g=40;
int high_h_g=95, high_s_g=255, high_v_g=255;


int low_h_g=55, low_s_g=80, low_v_g=101;
int high_h_g=95, high_s_g=255, high_v_g=255;
*/
int depth_high = 20;
int depth_high_limit=80;
int depth_low=70;

std::string color_type="HSV";
std::string circle_type="Poly";
Mat buffer;
Mat depth_mat_glo = Mat::zeros(1280, 720, CV_32F);
Mat result;
Mat result_fake;
ros::Publisher pub;
ros::Publisher pub_markers;

vector<vector<float> > circle_pix_all;
vector<vector<float> > circle_met_all;
vector<float> circle_depth_total_all;
vector<float> circle_depth_center_all;
vector<int> index_all;

vector<vector<float> > circle_pix_final;
vector<vector<float> > circle_met_final;
vector<float> circle_depth_final;

// Declaration of functions that changes int data to String
void morphOps(Mat &thresh) {
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
  Mat dilateElement = getStructuringElement( MORPH_RECT,Size(3,3));
  morphologyEx(thresh, thresh, MORPH_CLOSE, erodeElement);
  morphologyEx(thresh, thresh, MORPH_OPEN, erodeElement);
  //erode(thresh,thresh,erodeElement);
  //erode(thresh,thresh,erodeElement);
  //dilate(thresh,thresh,dilateElement);
  //dilate(thresh,thresh,dilateElement);
}

// Declaration of functions that calculates the ball position from pixel position
vector<float> pixel2point(Point center, int radius) {
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

string intToString(int n) {
  stringstream s;
  s << n;
  return s.str();
}

string floatToString(float f) {
  ostringstream buffer;
  buffer << f;
  return buffer.str();
}

void locator(int event, int x, int y, int flag, void *userdata) {
  if (event == EVENT_LBUTTONDOWN) cout<<"Left click has been made, Position: ("<<x<<","<<y<<")"<<endl;
}

Mat make_roi(Mat img, float x, float y, float r)
{
  int x_0=max(x-r+0.5,0.0);
  int y_0=max(y-r+0.5,0.0);
  int x_1=min(x+r+0.5,1279.9);
  int y_1=min(y+r+0.5,719.9);
  int s_x=x_1-x_0+1;
  int s_y=y_1-y_0+1;
  Mat roi_img=img(Rect(x_0, y_0, s_x, s_y));
  return roi_img;
}

vector<vector<float> > find_circle(Mat img)
{
  Mat img_raw;
  Mat img_raw_1;
  Mat img_raw_2;
  img_raw=img;
  int lowThreshold=100;
  int ratio=3;
  int kernel_size=3;
  Mat img_blur;
  Mat img_canny;
  Canny(img, img_canny, lowThreshold, lowThreshold*ratio, kernel_size);
  vector<Vec4i> hierarchy;
  vector<vector<Point> > contours;
  findContours(img_canny, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Point2f> center( contours.size() );
  vector<float> radius( contours.size() );
  vector<vector<float> > circle_pix;
  for( size_t i = 0; i < contours.size(); i++ )
  {
    approxPolyDP( contours[i], contours_poly[i], 1, true );
    minEnclosingCircle( contours_poly[i], center[i], radius[i] );
    if(radius[i]> iMin_tracking_ball_size)
    {
      vector<float> circle_local;
      circle_local.push_back(center[i].x);
      circle_local.push_back(center[i].y);
      circle_local.push_back(radius[i]-radcon);
      circle_pix.push_back(circle_local);
    }
  }
  return circle_pix;
}

vector<float> find_circle_depth_total(Mat dep_img, Mat inrange_img, vector<vector<float> > circle_pix)
{
  vector<float> ball_mean_depth;
  int real_ball=circle_pix.size();
  for (int i=0 ; i<real_ball; i++)
  {
    Scalar color = Scalar(0,0,255);
    Mat find_depth_img(Size(1280, 720), CV_8UC3, Scalar(0,0,0));
    float pix_x = circle_pix[i][0];
    float pix_y = circle_pix[i][1];
    float ball_rad = circle_pix[i][2];
    circle(find_depth_img, Point2f(pix_x, pix_y), (int)(ball_rad),color,CV_FILLED,8,0);
    Mat roi_mask=make_roi(find_depth_img, pix_x, pix_y, ball_rad);
    Mat roi_depth=make_roi(dep_img, pix_x, pix_y, ball_rad);
    Mat roi_inrange=make_roi(inrange_img, pix_x, pix_y, ball_rad);
    Mat mask;
    inRange(roi_mask, Scalar(0, 0, 254, 0), Scalar(0, 0, 255, 0), mask);
    Mat roi_bitwise_mask;
    bitwise_and(mask, roi_inrange, roi_bitwise_mask);
    Scalar mean_value;
    mean_value = mean(roi_depth, roi_bitwise_mask); //Mean distance value of red pixels
    float mean_val_raw = sum(mean_value)[0];
    float mean_val = roundf(mean_val_raw * 1000) / 1000;
    ball_mean_depth.push_back(mean_val);
  }
  return ball_mean_depth;
}

vector<float> find_circle_depth_center(Mat dep_img, Mat inrange_img, vector<vector<float> > circle_pix)
{
  vector<float> ball_mean_depth;
  int real_ball=circle_pix.size();
  for (int i=0 ; i<real_ball; i++)
  {
    Scalar color = Scalar(0,0,255);
    Mat find_depth_img(Size(1280, 720), CV_8UC3, Scalar(0,0,0));
    float pix_x = circle_pix[i][0];
    float pix_y = circle_pix[i][1];
    float ball_rad = circle_pix[i][2];
    circle(find_depth_img, Point2f(pix_x, pix_y) , (int)(ball_rad*0.5), color, CV_FILLED, 8, 0);
    Mat mask;
    inRange(find_depth_img, Scalar(0, 0, 254, 0), Scalar(0, 0, 255, 0), mask);
    Scalar mean_value;
    mean_value = mean(dep_img, mask); //Mean distance value of red pixels
    float mean_val_raw = sum(mean_value)[0];
    float mean_val = roundf(mean_val_raw * 1000) / 1000;
    ball_mean_depth.push_back(mean_val);
  }
  return ball_mean_depth;
}

vector<vector<float> > find_circle_meter(vector<vector<float> > circle_pix)
{
  vector<vector<float> > circle_met;
  int real_ball=circle_pix.size();
  for (int i=0; i<real_ball; i++)
  {
    float ball_pix_x=circle_pix[i][0];
    float ball_pix_y=circle_pix[i][1];
    float ball_radius_pix=circle_pix[i][2];
    vector<float> ball_position;
    ball_position= pixel2point(Point(ball_pix_x,ball_pix_y), (int)ball_radius_pix);
    circle_met.push_back(ball_position);
  }
  return circle_met;
}

vector<int> index_circle_same(vector<vector<float> >circle_pix)
{
  int real_ball=circle_pix.size();
  vector<int> index_z(real_ball);

  for (int i=0; i<real_ball ; i++)
  {
    int n=0;
    if(index_z[i]==-1)
    {
      continue;
    }
    for (int j=i+1; j<real_ball;j++)
    {
      float pix_x_i=circle_pix[i][0];
      float pix_y_i=circle_pix[i][1];
      float radius_pix_i=circle_pix[i][2];
      float pix_x_j=circle_pix[j][0];
      float pix_y_j=circle_pix[j][1];
      float radius_pix_j=circle_pix[j][2];
      float radius_pix_sum=radius_pix_i+radius_pix_j;
      float ball_dist_pix=pow(pix_x_i-pix_x_j,2)+pow(pix_y_i-pix_y_j,2);
      if(ball_dist_pix < pow(radius_pix_sum,2))
      {
        if(radius_pix_i < radius_pix_j)
        {
          index_z[i]=-1;
          n++;
          continue;
        }
        else
        {
          index_z[j]=-1;
        }
      }
    }
    if(n==0)
    {
      index_z[i]=0;
    }
  }
  vector<int> index_vec;
  for (int i=0;i<real_ball;i++)
  {
    index_vec.push_back(index_z[i]);
  }
  return index_vec;
}

vector<int> index_circle(vector<vector<float> > circle_met, vector<float> depth_total, vector<float> depth_center, vector<int> index)
{
  int real_ball=circle_met.size();
  if (real_ball!=depth_total.size())
  {
    cout<<"delete_circle_dep_error - input size"<<endl;
    return vector<int>();
  }
  vector<int> index_z(real_ball);
  for (int i=0; i<real_ball ; i++)
  {
    if (index[i]==-1)
    {
      index_z[i]=-1;
      continue;
    }

    if(depth_center[i]<0.25 || circle_met[i][2] > 5)
    {
      index_z[i]=-2;
      continue;
    }
    else if (circle_met[i][2]-max(0.3 * depth_center[i], (depth_low/100.0)) > depth_center[i])
    {
      if (circle_met[i][2]-max(0.3 * depth_total[i], (depth_low/100.0)) > depth_total[i])
      {
        index_z[i]=-3;
        continue;
      }
      index_z[i]=2;
      continue;
    }
    else if (circle_met[i][2]+max(0.3*depth_center[i], (depth_high/100.0))<depth_center[i])
    {
      if(circle_met[i][2]+max(0.3*depth_center[i], (depth_high/100.0))<depth_center[i] && circle_met[i][2]+max(depth_high_limit/100.0,depth_center[i]*1.7)>depth_center[i])
      {

        index_z[i]=3;
        continue;
      }
      else
      {
        index_z[i]=-5;
        continue;
      }
    }
    index_z[i]=1;
  }
  vector<int> index_vec;
  for (int i=0;i<real_ball;i++)
  {
    index_vec.push_back(index_z[i]);
  }
  return index_vec;
}

vector<vector<float> > sort_pix_met(vector<vector<float> > circle_pix_met, int def_num)
{
  int index_num=index_all.size();
  if (index_num != circle_pix_met.size())
  {
    cout<<"sort error-pix"<<endl;
    return vector<vector<float> >();
  }
  vector<vector<float> > circle_pix_met_sort;
  for (int i=0; i<index_num ; i++)
  {
    if(index_all[i]==def_num)
    {
      circle_pix_met_sort.push_back(circle_pix_met[i]);
    }
  }
  return circle_pix_met_sort;
}

vector<float> sort_depth(vector<float> depth, int def_num)
{
  int index_num=index_all.size();
  if (index_num != depth.size())
  {
    cout<<"sort error-depth"<<endl;
    return vector<float>();
  }
  vector<float> depth_sort;
  for (int i=0; i<index_num ; i++)
  {
    if(index_all[i]==def_num)
    {
      depth_sort.push_back(depth[i]);
    }
  }
  return depth_sort;
}

Mat roi_mask_first(Mat img, Mat img_depth, float x, float y, int r, float depth)
{
  Mat roi_img;
  Mat roi_depth;
  Mat roi_bitwise;
  roi_img=make_roi(img, x, y, r);
  roi_depth=make_roi(img_depth, x, y, r);
  Mat roi_depth_mask;
  inRange(roi_depth, depth-0.15, depth+0.15, roi_depth_mask);
  bitwise_and(roi_img, roi_depth_mask, roi_bitwise);
  return roi_bitwise;
}

Mat roi_mask_second(Mat img, Mat img_depth, float x, float y, int r, float depth, float roi_x, float roi_y, float roi_r)
{
  Mat roi_img;
  Mat roi_depth;
  Mat roi_bitwise;
  Mat roi_rest;
  Mat roi_rest_1;
  Scalar color = Scalar(0,0,255);
  roi_img=make_roi(img, x, y, r);
  Mat roi_img_zero(roi_img.size(), CV_8UC3, Scalar(0,0,0));
  circle(roi_img_zero, Point2f(roi_x,roi_y) , (int)(roi_r), color, CV_FILLED, 8, 0);
  Mat roi_mask;
  inRange(roi_img_zero, Scalar(0, 0, 254, 0), Scalar(0, 0, 255, 0), roi_mask);
  roi_rest_1=roi_img-roi_mask;
  /*
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(9,9));
  Mat roi_rest_2;
  morphologyEx(roi_rest_1, roi_rest_2, MORPH_CLOSE, erodeElement);
  morphologyEx(roi_rest_2, roi_rest, MORPH_OPEN, erodeElement);
  */
  return roi_rest_1;
}
/*
vector<vector<float> > overlay_ball_pix(Mat img, Mat img_depth, vector<vector<float> > circle_pix, vector<float> circle_depth)
{
  int ball=circle_pix.size();
  vector<vector<float> > overlay_circle_pix;
  for (int i=0; i<ball;i++)
  {
    float x = circle_pix[i][0];
    float y = circle_pix[i][1];
    float r = circle_pix[i][2];
    float depth=circle_depth[i];
    Mat roi_bitwise=roi_mask_first(img, img_depth, x, y ,r, depth);
    vector<vector<float> >circle_pix_fir = find_circle(roi_bitwise);
    vector<int> index_fir =index_circle_same(circle_pix_fir);
    vector<vector<float> > circle_pix_sort_fir=sort_pix_met(circle_pix_fir, index_fir, 0);

    for (int j=0; j<circle_pix_sort_fir.size(); j++)
    {
      vector<float> circle_pix_sort_fir_local;
      circle_pix_sort_fir_local.push_back(x-r+circle_pix_sort_fir[j][0]);
      circle_pix_sort_fir_local.push_back(y-r+circle_pix_sort_fir[j][1]);
      circle_pix_sort_fir_local.push_back(circle_pix_sort_fir[j][2]);
      overlay_circle_pix.push_back(circle_pix_sort_fir_local);
    }
    //imshow("roi_bitwise",roi_bitwise);
    cout<<circle_pix_sort_fir.size()<<endl;
    if (circle_pix_sort_fir.size()>0)
    {
      Mat roi_rest=roi_mask_second(img, img_depth, x, y ,r, depth, circle_pix_sort_fir[0][0], circle_pix_sort_fir[0][1],circle_pix_sort_fir[0][2]);
      vector<vector<float> >circle_pix_sec = find_circle(roi_rest);
      vector<int> index_sec =index_circle_same(circle_pix_sec);
      vector<vector<float> > circle_pix_sort_sec=sort_pix_met(circle_pix_sec, index_sec, 0);

      for (int j=0; j<circle_pix_sort_sec.size(); j++)
      {
        circle_pix_sort_sec[j][0]+=x;
        circle_pix_sort_sec[j][1]+=y;
        overlay_circle_pix.push_back(circle_pix_sort_sec[j]);
      }
    }
  }

  return overlay_circle_pix;
}
*/
vector<vector<float> > fus_circle(vector<vector<float> > circle_1, vector<vector<float> > circle_2, vector<vector<float> > circle_3)
{
  vector<vector<float> > circle_fus_pix;
  for (int i=0; i<circle_1.size();i++)
  {
    circle_fus_pix.push_back(circle_1[i]);
  }
  cout<<"size: "<<circle_fus_pix.size()<<endl;
  for (int i=0; i<circle_2.size(); i++)
  {
    if (circle_fus_pix.size()<3)
    {

      circle_fus_pix.push_back(circle_2[i]);
    }
  }
  for (int i=0; i<circle_3.size();i++)
  {
    if (circle_fus_pix.size()<3)
    {
      circle_fus_pix.push_back(circle_3[i]);
    }
  }
  return circle_fus_pix;
    //int ind=index[i];
}

vector<float> fus_circle_depth(vector<float> circle_1, vector<float> circle_2, vector<float> circle_3)
{
  vector<float> circle_fus_depth;
  for (int i=0; i<circle_1.size();i++)
  {
    circle_fus_depth.push_back(circle_1[i]);
  }
  cout<<"depth_size: "<<circle_fus_depth.size()<<endl;
  for (int i=0; i<circle_2.size(); i++)
  {
    if (circle_fus_depth.size()<3)
    {
      circle_fus_depth.push_back(circle_2[i]);
    }
  }
  for (int i=0; i<circle_3.size();i++)
  {
    if (circle_fus_depth.size()<3)
    {
      circle_fus_depth.push_back(circle_3[i]);
    }
  }
  return circle_fus_depth;
}

vector<vector<float> > find_real_meter(vector<vector<float> > circle_met, vector<float> circle_depth)
{
  int real_ball=circle_met.size();
  vector<vector<float> > carpos_met;
  for(int k=0; k<real_ball; k++)
  {
    vector<float> carpos_met_local;
    carpos_met_local.push_back(circle_met[k][0]);
    carpos_met_local.push_back((circle_depth[k]*cos(theta* PI / 180.0)-circle_met[k][1]*sin(theta* PI / 180.0)));
    carpos_met.push_back(carpos_met_local);
  }
  return carpos_met;
}

void draw_circle(Mat result, vector<vector<float> > circle_pix, vector<vector<float> > circle_met, vector<float> circle_depth, Scalar color)
{
  int ball_num = circle_met.size();
  string ball_color;
  if(color==Scalar(255, 0, 0))
  {
    ball_color="Blue ball: ";
  }
  else if(color==Scalar(0,255,0))
  {
    ball_color="Green ball: ";
  }
  else if(color==Scalar(0,0,255))
  {
    ball_color="Red ball: ";
  }
  for (int i=0; i<ball_num;i++)
  {
    float isx = circle_met[i][0];
    float isy = circle_met[i][1];
    float isz = circle_met[i][2];
    float isr = circle_pix[i][2];
    float isdepth = circle_depth[i];
    //int ind=index[i];
    string sx = floatToString(isx);
    string sy = floatToString(isy);
    string sz = floatToString(isz);
    string sr = floatToString(isr);
    string sdepth = floatToString(isdepth);
    //string inds=intToString(ind);
  //text = ball_color + sx + "," + sy + "," + sz + "," + sr + "," + sdepth;
    text = sz + "," + sdepth;

    putText(result, text, Point2f(circle_pix[i][0],circle_pix[i][1]),2,1,Scalar(0,255,0),2);
    //putText(result, "Real", Point2f(circle_pix[i][0],circle_pix[i][1]),2,1,Scalar(0,255, 0),2);
    circle(result, Point2f(circle_pix[i][0],circle_pix[i][1]), (int)circle_pix[i][2], color, 2, 8, 0 );
  }
}

void draw_fake_circle(Mat result, vector<vector<float> > circle_pix, vector<vector<float> > circle_met, vector<float> circle_depth_center, vector<float> circle_depth_total, Scalar color, vector<int> index)
{
  int ball_num = circle_met.size();
  string ball_color;
  if(color==Scalar(255, 0, 0))
  {
    ball_color="Blue ball:";
  }
  else if(color==Scalar(0,255,0))
  {
    ball_color="Green ball:";
  }
  else if(color==Scalar(0,0,255))
  {
    ball_color="Red ball:";
  }
  for (int i=0; i<ball_num;i++)
  {
    float isx = circle_met[i][0];
    float isy = circle_met[i][1];
    float isz = circle_met[i][2];
    float isdepth_center=circle_depth_center[i];
    float isdepth_total=circle_depth_total[i];
    int ind=index[i];
    string sx = floatToString(isx);
    string sy = floatToString(isy);
    string sz = floatToString(isz);
    string inds=intToString(ind);
    string sdepth_center = floatToString(isdepth_center);
    string sdepth_total = floatToString(isdepth_total);
    //text = ball_color + sx + "," + sy + "," + sz+","+sdepth_center+"," + sdepth_total + ","+inds;
    text = sz+","+sdepth_center+"," + sdepth_total;
    //putText(result, text, Point2f(circle_pix[i][0],circle_pix[i][1]),2,1,Scalar(0,0,255),2);
    putText(result, text, Point2f(circle_pix[i][0],circle_pix[i][1]),2,1,Scalar(0,0,255),2);
    circle(result, Point2f(circle_pix[i][0],circle_pix[i][1]), (int)circle_pix[i][2], color, 2, 8, 0 );
  }
}

int image_to_ball_all(Mat img, Mat depth_img)
{
  circle_pix_all=find_circle(img);
  circle_met_all=find_circle_meter(circle_pix_all);
  circle_depth_total_all=find_circle_depth_total(depth_img, img, circle_pix_all);
  circle_depth_center_all=find_circle_depth_center(depth_img, img, circle_pix_all);
  vector<int> index_same;
  index_same=index_circle_same(circle_pix_all);
  index_all=index_circle(circle_met_all, circle_depth_total_all, circle_depth_center_all, index_same);
  return 0;
}

int ball_all_to_ball_final(Scalar color)
{
  vector<vector<float> > circle_pix_centered;
  vector<vector<float> > circle_met_centered;
  vector<float> circle_depth_centered;
  circle_pix_centered=sort_pix_met(circle_pix_all, 1);
  circle_met_centered=sort_pix_met(circle_met_all, 1);
  circle_depth_centered=sort_depth(circle_depth_center_all, 1);
  vector<vector<float> > circle_pix_totald;
  vector<vector<float> > circle_met_totald;
  vector<float> circle_depth_totald;
  circle_pix_totald=sort_pix_met(circle_pix_all, 2);
  circle_met_totald=sort_pix_met(circle_met_all, 2);
  circle_depth_totald=sort_depth(circle_depth_total_all, 2);
  vector<vector<float> > circle_pix_over;
  vector<vector<float> > circle_pix_over_raw;
  vector<vector<float> > circle_met_over;
  vector<vector<float> > circle_met_over_raw;
  vector<float> circle_depth_over;
  /*
  vector<vector<float> > circle_pix_over_raw;
  vector<float> circle_depth_over_raw;
  */
  if (color == Scalar(255, 0, 0))
  {
    float x_cen=959;
    float y_cen=539;
    circle_pix_over_raw=sort_pix_met(circle_pix_all, 3);
    circle_met_over_raw=sort_pix_met(circle_met_all, 3);
    circle_depth_over=sort_depth(circle_depth_center_all, 3);
    int size_3=circle_met_over_raw.size();
    for (int i=0; i<size_3; i++)
    {
      vector<float> circle_met_over_local;
      vector<float> circle_pix_over_local;
      for (int j=0; j<3; j++)
      {
        if(j<2)
        {
          circle_pix_over_local.push_back((circle_pix_over_raw[i][j]));
        }
        else
        {
          circle_pix_over_local.push_back((circle_pix_over_raw[i][j])*(circle_met_over_raw[i][2])/(circle_depth_over[i]));
        }
        circle_met_over_local.push_back((circle_met_over_raw[i][j])*(circle_depth_over[i])/(circle_met_over_raw[i][2]));
      }
      circle_pix_over.push_back(circle_pix_over_local);
      circle_met_over.push_back(circle_met_over_local);
      vector<float> ().swap(circle_pix_over_local);
      vector<float> ().swap(circle_met_over_local);
    }
  }
  circle_pix_final=fus_circle(circle_pix_centered, circle_pix_totald, circle_pix_over);
  circle_met_final=fus_circle(circle_met_centered, circle_met_totald, circle_met_over);
  circle_depth_final=fus_circle_depth(circle_depth_centered, circle_depth_totald, circle_depth_over);
  return 0;
}


vector<vector<float> > ball_final_to_ball_pos()
{
  vector<vector<float> > carpos;
  carpos=find_real_meter(circle_met_final, circle_depth_final);
  return carpos;
}

void on_trackbar_low_h2_r(int, void*)
{
  low_h2_r=min(high_h2_r-1, low_h2_r);
  setTrackbarPos("Low H2_R","result_red",low_h2_r);
}

void on_trackbar_high_h2_r(int, void*)
{
  high_h2_r=max(high_h2_r, low_h2_r+1);
  setTrackbarPos("High H2_R","result_red",high_h2_r);
}

void on_trackbar_low_h_r(int, void*)
{
  low_h_r=min(high_h_r-1, low_h_r);
  setTrackbarPos("Low H_R","result_red",low_h_r);
}

void on_trackbar_high_h_r(int, void*)
{
  high_h_r=max(high_h_r, low_h_r+1);
  setTrackbarPos("High H_R","result_red",high_h_r);
}

void on_trackbar_low_s_r(int, void*)
{
  low_s_r=min(high_s_r-1, low_s_r);
  setTrackbarPos("Low S_R","result_red",low_s_r);
}

void on_trackbar_high_s_r(int, void*)
{
  high_s_r=max(high_s_r, low_s_r+1);
  setTrackbarPos("High H_R","result_red",high_s_r);
}

void on_trackbar_low_v_r(int, void*)
{
  low_v_r=min(high_v_r-1, low_v_r);
  setTrackbarPos("Low V_R","result_red",low_v_r);
}

void on_trackbar_high_v_r(int, void*)
{
  high_v_r=max(high_v_r, low_v_r+1);
  setTrackbarPos("High V_R","result_red",high_v_r);
}
////////////////////////////////////////////////////////////

void on_trackbar_low_h_b(int, void*)
{
  low_h_b=min(high_h_b-1, low_h_b);
  setTrackbarPos("Low H_B","result_blue",low_h_b);
}

void on_trackbar_high_h_b(int, void*)
{
  high_h_b=max(high_h_b, low_h_b+1);
  setTrackbarPos("High H_B","result_blue",high_h_b);
}

void on_trackbar_low_s_b(int, void*)
{
  low_s_b=min(high_s_b-1, low_s_b);
  setTrackbarPos("Low S_B","result_blue",low_s_b);
}

void on_trackbar_high_s_b(int, void*)
{
  high_s_b=max(high_s_b, low_s_b+1);
  setTrackbarPos("High H_B","result_blue",high_s_b);
}

void on_trackbar_low_v_b(int, void*)
{
  low_v_b=min(high_v_b-1, low_v_b);
  setTrackbarPos("Low V_B","result_blue",low_v_b);
}

void on_trackbar_high_v_b(int, void*)
{
  high_v_b=max(high_v_b, low_v_b+1);
  setTrackbarPos("High V_B","result_blue",high_v_b);
}
//////////////////////////////////////////////////////////////

void on_trackbar_low_h_g(int, void*)
{
  low_h_g=min(high_h_g-1, low_h_g);
  setTrackbarPos("Low H_G","result_green",low_h_g);
}

void on_trackbar_high_h_g(int, void*)
{
  high_h_g=max(high_h_g, low_h_g+1);
  setTrackbarPos("High H_G","result_green",high_h_g);
}

void on_trackbar_low_s_g(int, void*)
{
  low_s_g=min(high_s_g-1, low_s_g);
  setTrackbarPos("Low S_G","result_green",low_s_g);
}

void on_trackbar_high_s_g(int, void*)
{
  high_s_g=max(high_s_g, low_s_g+1);
  setTrackbarPos("High S_G","result_green",high_s_g);
}

void on_trackbar_low_v_g(int, void*)
{
  low_v_g=min(high_v_g-1, low_v_g);
  setTrackbarPos("Low V_G","result_green",low_v_g);
}

void on_trackbar_high_v_g(int, void*)
{
  high_v_g=max(high_v_g, low_v_g+1);
  setTrackbarPos("High V_G","result_green",high_v_g);
}

void on_trackbar_depth_high(int, void*)
{
  depth_high=min(depth_high, depth_high_limit-1);
  setTrackbarPos("depth_high", "result", depth_high);
}

void on_trackbar_depth_high_limit(int, void*)
{
  depth_high_limit=max(depth_high+1, depth_high_limit);
  setTrackbarPos("depth_high_limit", "result", depth_high_limit);
}


void on_trackbar_depth_low(int, void*)
{
  setTrackbarPos("depth_low", "result", depth_low);
}



void ball_detect() {

  Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);
  Mat frame;
  if(buffer.size().width==320) cv::resize(buffer, frame, cv::Size(640, 480)); //if the size of the image is 320x240, then resized it to 640x480
  else frame = buffer;

  Mat calibrated_frame_size;
  Mat calibrated_frame;
  cv::resize(buffer, calibrated_frame_size ,cv::Size(1280,720));
  rectangle(calibrated_frame_size, Point(0,0), Point(1279,99), Scalar(0,0,0),-1);
  undistort(calibrated_frame_size, calibrated_frame, intrinsic, distCoeffs);
  //cv::resize(calibrated_frame_size, calibrated_frame, cv::Size(1280, 720));
  Mat hsv_frame;
  Mat hsv_frame_red;
  Mat hsv_frame_red1;
  Mat hsv_frame_red2;
  Mat hsv_frame_blue;
  Mat hsv_frame_green;

  result=calibrated_frame.clone();
  result_fake=calibrated_frame.clone();


  cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);
  inRange(hsv_frame, Scalar(low_h_r,low_s_r,low_v_r), Scalar(high_h_r,high_s_r,high_v_r), hsv_frame_red1);
  inRange(hsv_frame, Scalar(low_h2_r,low_s_r,low_v_r), Scalar(high_h2_r,high_s_r,high_v_r), hsv_frame_red2);
  inRange(hsv_frame, Scalar(low_h_b,low_s_b,low_v_b), Scalar(high_h_b,high_s_b,high_v_b), hsv_frame_blue);
  inRange(hsv_frame, Scalar(low_h_g,low_s_g,low_v_g), Scalar(high_h_g,high_s_g,high_v_g), hsv_frame_green);
  addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,5));

  Mat hsv_frame_red_1, hsv_frame_red_2, hsv_frame_blue_1, hsv_frame_blue_2, hsv_frame_green_1, hsv_frame_green_2;
  morphologyEx(hsv_frame_red, hsv_frame_red_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_red_1, hsv_frame_red_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_red_2, hsv_frame_red, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_green, hsv_frame_green_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_green_1, hsv_frame_green_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_green_2, hsv_frame_green, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_blue, hsv_frame_blue_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_blue_1, hsv_frame_blue_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_blue_2, hsv_frame_blue, cv::Size(9, 9), 2, 2);

  /*
  morphOps(hsv_frame_red);
  morphOps(hsv_frame_blue);
  morphOps(hsv_frame_green);

  Mat frame_red_blur;
  Mat frame_blue_blur;
  Mat frame_green_blur;
  Mat frame_red_canny;
  Mat frame_blue_canny;
  Mat frame_green_canny;

  GaussianBlur(hsv_frame_red, frame_red_blur, cv::Size(9, 9), 2, 2);
  GaussianBlur(hsv_frame_blue, frame_blue_blur, cv::Size(9, 9), 2, 2);
  GaussianBlur(hsv_frame_green, frame_green_blur, cv::Size(9, 9), 2, 2);
  Canny(frame_red_blur, frame_red_canny, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
  Canny(frame_blue_blur, frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);
  Canny(frame_green_blur, frame_green_canny, lowThreshold_g, lowThreshold_g*ratio_g, kernel_size_g);
  */
  Mat depth_frame = Mat::zeros(1280, 720, CV_32F);
  depth_frame = depth_mat_glo;
  //cv::resize(depth_mat_glo, depth_frame, cv::Size(1920, 1080));

  //imshow("hsv_red", hsv_frame_red);
  //imshow("hsv_blue", hsv_frame_blue);
  //imshow("hsv_green", hsv_frame_green);

  Scalar color_r = Scalar(0,0,255);
  Scalar color_b = Scalar(255,0,0);
  Scalar color_g = Scalar(0,255,0);


  vector<vector<float> > carpos_r;
  image_to_ball_all(hsv_frame_red, depth_frame);
  ball_all_to_ball_final(color_r);
  carpos_r=ball_final_to_ball_pos();



  vector<vector<float> > circle_pix_all_r = circle_pix_all;
  vector<vector<float> > circle_met_all_r = circle_met_all;
  vector<float> circle_depth_total_all_r = circle_depth_total_all;
  vector<float> circle_depth_center_all_r = circle_depth_center_all;
  vector<int> index_all_r = index_all;
  vector<vector<float> > circle_pix_final_r = circle_pix_final;
  vector<vector<float> > circle_met_final_r = circle_met_final;
  vector<float> circle_depth_final_r = circle_depth_final;
  draw_circle(result, circle_pix_final_r, circle_met_final_r, circle_depth_final_r, color_r);
  draw_fake_circle(result_fake, circle_pix_all_r, circle_met_all_r, circle_depth_center_all_r, circle_depth_total_all_r, color_r, index_all_r);


  vector<vector<float> > ().swap(circle_pix_all);
  vector<vector<float> > ().swap(circle_met_all);
  vector<float> ().swap(circle_depth_total_all);
  vector<float> ().swap(circle_depth_center_all);
  vector<int> ().swap(index_all);
  vector<vector<float> > ().swap(circle_pix_final);
  vector<vector<float> > ().swap(circle_met_final);
  vector<float> ().swap(circle_depth_final);
  depth_low=20;
  depth_high = 85;
  depth_high_limit=400;

  // vector<vector<float> > carpos_g;
  // image_to_ball_all(hsv_frame_green, depth_frame);
  // ball_all_to_ball_final(color_g);
  // carpos_g=ball_final_to_ball_pos();


  // vector<vector<float> > circle_pix_all_g = circle_pix_all;
  // vector<vector<float> > circle_met_all_g = circle_met_all;
  // vector<float> circle_depth_total_all_g = circle_depth_total_all;
  // vector<float> circle_depth_center_all_g = circle_depth_center_all;
  // vector<int> index_all_g = index_all;
  // vector<vector<float> > circle_pix_final_g = circle_pix_final;
  // vector<vector<float> > circle_met_final_g = circle_met_final;
  // vector<float> circle_depth_final_g = circle_depth_final;
  //
  // draw_circle(result, circle_pix_final_g, circle_met_final_g, circle_depth_final_g, color_g);
  // draw_fake_circle(result_fake, circle_pix_all_g, circle_met_all_g, circle_depth_center_all_g, circle_depth_total_all_g, color_g, index_all_g);
  depth_high = 20;
  depth_high_limit=80;
  depth_low=70;
  vector<vector<float> > ().swap(circle_pix_all);
  vector<vector<float> > ().swap(circle_met_all);
  vector<float> ().swap(circle_depth_total_all);
  vector<float> ().swap(circle_depth_center_all);
  vector<int> ().swap(index_all);
  vector<vector<float> > ().swap(circle_pix_final);
  vector<vector<float> > ().swap(circle_met_final);
  vector<float> ().swap(circle_depth_final);

  vector<vector<float> > carpos_b;
  image_to_ball_all(hsv_frame_blue, depth_frame);
  ball_all_to_ball_final(color_b);
  carpos_b=ball_final_to_ball_pos();

  vector<vector<float> > circle_pix_all_b = circle_pix_all;
  vector<vector<float> > circle_met_all_b = circle_met_all;
  vector<float> circle_depth_total_all_b = circle_depth_total_all;
  vector<float> circle_depth_center_all_b = circle_depth_center_all;
  vector<int> index_all_b = index_all;
  vector<vector<float> > circle_pix_final_b = circle_pix_final;
  vector<vector<float> > circle_met_final_b = circle_met_final;
  vector<float> circle_depth_final_b = circle_depth_final;
  draw_circle(result, circle_pix_final_b, circle_met_final_b, circle_depth_final_b, color_b);
  draw_fake_circle(result_fake, circle_pix_all_b, circle_met_all_b, circle_depth_center_all_b, circle_depth_total_all_b, color_b, index_all_b);


  core_msgs::ball_position msg;
  int real_ball_r=carpos_r.size();
  int real_ball_b=carpos_b.size();
  int real_ball_num=real_ball_r+real_ball_b;
  // int real_ball_g=carpos_g.size();
  cout<<"rb :"<<real_ball_r<<endl;
  cout<<"bb :"<<real_ball_b<<endl;
  // cout<<"gb :"<<real_ball_g<<endl;
  msg.r_size = real_ball_r;
  msg.r_img_x.resize(real_ball_r);
  msg.r_img_y.resize(real_ball_r);
  msg.b_size = real_ball_b;
  msg.b_img_x.resize(real_ball_b);
  msg.b_img_y.resize(real_ball_b);
  msg.size = real_ball_num;
  msg.img_x.resize(real_ball_num);
  msg.img_y.resize(real_ball_num);
  // msg.g_size = real_ball_g;
  // msg.g_img_x.resize(real_ball_g);
  // msg.g_img_y.resize(real_ball_g);
  for(int k=0; k<real_ball_r; k++)
  {
    msg.r_img_x[k] = carpos_r[k][0];
    msg.r_img_y[k] = carpos_r[k][1];
    msg.img_x[k]=carpos_r[k][0];
    msg.img_y[k]=carpos_r[k][1];
  }
  for(int k=0; k<real_ball_b; k++)
  {
    msg.b_img_x[k] = carpos_b[k][0];
    msg.b_img_y[k] = carpos_b[k][1];
    msg.img_x[k+real_ball_r] = carpos_b[k][0];
    msg.img_y[k+real_ball_r] = carpos_b[k][1];
  }

  // for(int k=0; k<real_ball_g; k++)
  // {
  //   msg.g_img_x[k] = carpos_g[k][0];
  //   msg.g_img_y[k] = carpos_g[k][1];
  // }

  if (real_ball_b==3)
  {
    msg.b_over = -1;
  }

  if(real_ball_b==2)
  {
    msg.b_over=1;
  }


  pub.publish(msg);  //publish a message

  vector<vector<float> > ().swap(circle_pix_all);
  vector<vector<float> > ().swap(circle_met_all);
  vector<float> ().swap(circle_depth_total_all);
  vector<float> ().swap(circle_depth_center_all);
  vector<int> ().swap(index_all);
  vector<vector<float> > ().swap(circle_pix_final);
  vector<vector<float> > ().swap(circle_met_final);
  vector<float> ().swap(circle_depth_final);




  namedWindow("result", WINDOW_NORMAL);

  //createTrackbar("high depth","result", &depth_high, 500, on_trackbar_depth_high);
  //createTrackbar("high depth limit", "result", &depth_high_limit, 1000, on_trackbar_depth_high_limit);
  //createTrackbar("low depth","result", &depth_low, 500, on_trackbar_depth_low);

  imshow("result",result);

  //namedWindow("result-fake", WINDOW_NORMAL);
  //imshow("result-fake",result_fake);
///////////////////////////

/*

  Mat result_red=calibrated_frame.clone();
  Mat result_fake_red=calibrated_frame.clone();
  namedWindow("result_red", WINDOW_NORMAL);
  namedWindow("result_fake_red", WINDOW_NORMAL);
  createTrackbar("Low H2_R","result_red", &low_h2_r, 180, on_trackbar_low_h_r);
  createTrackbar("High H2_R","result_red", &high_h2_r, 180, on_trackbar_high_h_r);
  createTrackbar("Low H_R","result_red", &low_h_r, 180, on_trackbar_low_h_r);
  createTrackbar("High H_R","result_red", &high_h_r, 180, on_trackbar_high_h_r);
  createTrackbar("Low S_R","result_red", &low_s_r, 255, on_trackbar_low_s_r);
  createTrackbar("High S_R","result_red", &high_s_r, 255, on_trackbar_high_s_r);
  createTrackbar("Low V_R","result_red", &low_v_r, 255, on_trackbar_low_v_r);
  createTrackbar("High V_R","result_red", &high_v_r, 255, on_trackbar_high_v_r);
  draw_circle(result_red, circle_pix_final_r, circle_met_final_r, circle_depth_final_r, Scalar(0,0,255));
  draw_fake_circle(result_fake_red, circle_pix_all_r, circle_met_all_r, circle_depth_center_all_r, circle_depth_total_all_r, Scalar(0,0,255), index_all_r);
  imshow("result_red", result_red);
  imshow("result_fake_red", result_fake_red);

*/
  ///////////////////////////
/*
  Mat result_green=calibrated_frame.clone();
  Mat result_fake_green=calibrated_frame.clone();
  namedWindow("result_green", WINDOW_NORMAL);
  namedWindow("result_fake_green", WINDOW_NORMAL);
  createTrackbar("Low H_G","result_green", &low_h_g, 180, on_trackbar_low_h_g);
  createTrackbar("High H_G","result_green", &high_h_g, 180, on_trackbar_high_h_g);
  createTrackbar("Low S_G","result_green", &low_s_g, 255, on_trackbar_low_s_g);
  createTrackbar("High S_G","result_green", &high_s_g, 255, on_trackbar_high_s_g);
  createTrackbar("Low V_G","result_green", &low_v_g, 255, on_trackbar_low_v_g);
  createTrackbar("High V_G","result_green", &high_v_g, 255, on_trackbar_high_v_g);
  draw_circle(result_green, circle_pix_final_g, circle_met_final_g, circle_depth_final_g, Scalar(0,255,0));
  draw_fake_circle(result_fake_green, circle_pix_all_g, circle_met_all_g, circle_depth_center_all_g, circle_depth_total_all_g, Scalar(0,255,0), index_all_g);
  imshow("result_green", result_green);
  imshow("result_fake_green", result_fake_green);
*/
////////////////////////////////////
/*
  Mat result_blue=calibrated_frame.clone();
  Mat result_fake_blue=calibrated_frame.clone();
  namedWindow("result_blue", WINDOW_NORMAL);
  namedWindow("result_fake_blue", WINDOW_NORMAL);
  createTrackbar("Low H_B","result_blue", &low_h_b, 180, on_trackbar_low_h_b);
  createTrackbar("High H_B","result_blue", &high_h_b, 180, on_trackbar_high_h_b);
  createTrackbar("Low S_B","result_blue", &low_s_b, 255, on_trackbar_low_s_b);
  createTrackbar("High S_B","result_blue", &high_s_b, 255, on_trackbar_high_s_b);
  createTrackbar("Low V_B","result_blue", &low_v_b, 255, on_trackbar_low_v_b);
  createTrackbar("High V_B","result_blue", &high_v_b, 255, on_trackbar_high_v_b);
  draw_circle(result_blue, circle_pix_final_b, circle_met_final_b, circle_depth_final_b, Scalar(255,0,0));
  draw_fake_circle(result_fake_blue, circle_pix_all_b, circle_met_all_b, circle_depth_center_all_b, circle_depth_total_all_b, Scalar(255,0,0), index_all_b);
  imshow("result_blue", result_blue);
  imshow("result_fake_blue", result_fake_blue);

*/
/*
  vector<vector<float> > ().swap(circle_pix_all_r);
  vector<vector<float> > ().swap(circle_met_all_r);
  vector<float> ().swap(circle_depth_total_all_r);
  vector<float> ().swap(circle_depth_center_all_r);
  vector<int> ().swap(index_all_r);
  vector<vector<float> > ().swap(circle_pix_final_r);
  vector<vector<float> > ().swap(circle_met_final_r);
  vector<float> ().swap(circle_depth_final_r);

  vector<vector<float> > ().swap(circle_pix_all_g);
  vector<vector<float> > ().swap(circle_met_all_g);
  vector<float> ().swap(circle_depth_total_all_g);
  vector<float> ().swap(circle_depth_center_all_g);
  vector<int> ().swap(index_all_g);
  vector<vector<float> > ().swap(circle_pix_final_g);
  vector<vector<float> > ().swap(circle_met_final_g);
  vector<float> ().swap(circle_depth_final_g);

  vector<vector<float> > ().swap(circle_pix_all_b);
  vector<vector<float> > ().swap(circle_met_all_b);
  vector<float> ().swap(circle_depth_total_all_b);
  vector<float> ().swap(circle_depth_center_all_b);
  vector<int> ().swap(index_all_b);
  vector<vector<float> > ().swap(circle_pix_final_b);
  vector<vector<float> > ().swap(circle_met_final_b);
  vector<float> ().swap(circle_depth_final_b);

*/
  /*

  vector<Vec4i> hierarchy_r;
  vector<Vec4i> hierarchy_b;
  vector<Vec4i> hierarchy_g;
  vector<vector<Point> > contours_r;
  vector<vector<Point> > contours_b;
  vector<vector<Point> > contours_g;
  findContours(frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  findContours(frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
  findContours(frame_green_canny, contours_g, hierarchy_g, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<vector<Point> > contours_r_poly( contours_r.size() );
  vector<vector<Point> > contours_b_poly( contours_b.size() );
  vector<vector<Point> > contours_g_poly( contours_g.size() );
  vector<Point2f> center_r( contours_r.size() );
  vector<Point2f> center_b( contours_b.size() );
  vector<Point2f> center_g( contours_g.size() );
  vector<float> radius_r( contours_r.size() );
  vector<float> radius_b( contours_b.size() );
  vector<float> radius_g( contours_g.size() );

  for( size_t i = 0; i < contours_r.size(); i++ ) {
    approxPolyDP( contours_r[i], contours_r_poly[i], 1, true );
    minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] );
  }
  for( size_t i = 0; i < contours_b.size(); i++ ) {
    approxPolyDP( contours_b[i], contours_b_poly[i], 1, true );
    minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
  }
  for( size_t i = 0; i < contours_g.size(); i++ ) {
    approxPolyDP( contours_g[i], contours_g_poly[i], 1, true );
    minEnclosingCircle( contours_g_poly[i], center_g[i], radius_g[i] );
  }

  int real_ball_r = 0;
  int* real_ball_radius_pix_r = new int[contours_r.size()];
  int* real_ball_pix_r_x=new int[contours_r.size()];
  int* real_ball_pix_r_y=new int[contours_r.size()];
  float** carpos_r = new float*[contours_r.size()];
  for(int j = 0; j < contours_r.size(); ++j) carpos_r[j] = new float[3];
  for( size_t i = 0; i< contours_r.size(); i++ ) {
    bool diff_ball_r=true;
    if (radius_r[i] > iMin_tracking_ball_size) {
      int pix_x_r=center_r[i].x;
      int pix_y_r=center_r[i].y;
      int radius_pix_r=((int)radius_r[i]-radcon);
      for (int j=0 ; j<real_ball_r ; j++) {
        int real_pix_x_r=real_ball_pix_r_x[j];
        int real_pix_y_r=real_ball_pix_r_y[j];
        int radius_sum_r=real_ball_radius_pix_r[j]+radius_pix_r;
        int ball_dist_pix_r=pow(pix_x_r-real_pix_x_r,2)+pow(pix_y_r-real_pix_y_r,2);
        if(ball_dist_pix_r < pow(radius_sum_r,2)) {
          if (radius_pix_r>real_ball_radius_pix_r[j]) carpos_r[j][2]=-1;
          else diff_ball_r=false;
        }
      }
      if (diff_ball_r) {
        Scalar color = Scalar( 0, 0, 255);
        vector<float> ball_position_r;
        radius_r[i]=radius_r[i]-radcon;
        ball_position_r = pixel2point(center_r[i], radius_r[i]);
        float isx = ball_position_r[0];
        float isy = ball_position_r[1];
        float isz = ball_position_r[2];
        carpos_r[real_ball_r][0] = isx;
        carpos_r[real_ball_r][1] = isy;
        carpos_r[real_ball_r][2] = isz;
        real_ball_radius_pix_r[real_ball_r]=radius_r[i];
        real_ball_pix_r_x[real_ball_r]=center_r[i].x;
        real_ball_pix_r_y[real_ball_r]=center_r[i].y;
        real_ball_r++;
      }
    }
  }
  int real_ball_b = 0;
  int* real_ball_radius_pix_b = new int[contours_b.size()];
  int* real_ball_pix_b_x = new int[contours_b.size()];
  int* real_ball_pix_b_y = new int[contours_b.size()];
  float** carpos_b = new float*[contours_b.size()];
  for(int j = 0; j < contours_b.size(); ++j) carpos_b[j] = new float[3];
  for( size_t i = 0; i< contours_b.size(); i++ ) {
    bool diff_ball_b=true;
    if (radius_b[i] > iMin_tracking_ball_size) {
      int pix_x_b=center_b[i].x;
      int pix_y_b=center_b[i].y;
      int radius_pix_b=(radius_b[i]-radcon);
      for (int j=0 ; j<real_ball_b ; j++) {
        int real_pix_x_b=real_ball_pix_b_x[j];
        int real_pix_y_b=real_ball_pix_b_y[j];
        int radius_sum_b=real_ball_radius_pix_b[j]+radius_pix_b;
        int ball_dist_pix_b=pow(pix_x_b-real_pix_x_b,2)+pow(pix_y_b-real_pix_y_b,2);
        if(ball_dist_pix_b < pow(radius_sum_b,2)) {
          if (radius_b[i]>real_ball_radius_pix_b[j]) carpos_b[j][2]=-1;
          else diff_ball_b=false;
        }
      }
      if (diff_ball_b) {
        Scalar color = Scalar( 255, 0, 0);
        vector<float> ball_position_b;
        radius_b[i]=radius_b[i]-radcon;
        ball_position_b = pixel2point(center_b[i], radius_b[i]);
        float isx = ball_position_b[0];
        float isy = ball_position_b[1];
        float isz = ball_position_b[2];
        carpos_b[real_ball_b][0] = isx;
        carpos_b[real_ball_b][1] = isy;
        carpos_b[real_ball_b][2] = isz;
        real_ball_radius_pix_b[real_ball_b]=radius_b[i];
        real_ball_pix_b_x[real_ball_b]=center_b[i].x;
        real_ball_pix_b_y[real_ball_b]=center_b[i].y;
        real_ball_b++;
      }
    }
  }
  int real_ball_g = 0;
  int* real_ball_radius_pix_g = new int[contours_g.size()];
  int* real_ball_pix_g_x = new int[contours_g.size()];
  int* real_ball_pix_g_y = new int[contours_g.size()];
  float** carpos_g = new float*[contours_g.size()];
  for(int j = 0; j < contours_g.size(); ++j) carpos_g[j] = new float[3];
  for( size_t i = 0; i< contours_g.size(); i++ ) {
    bool diff_ball_g=true;
    if (radius_g[i] > iMin_tracking_ball_size) {
      int pix_x_g=center_g[i].x;
      int pix_y_g=center_g[i].y;
      int radius_pix_g=(radius_g[i]-radcon);
      for (int j=0 ; j<real_ball_g ; j++) {
        int real_pix_x_g=real_ball_pix_g_x[j];
        int real_pix_y_g=real_ball_pix_g_y[j];
        int radius_sum_g=real_ball_radius_pix_g[j]+radius_pix_g;
        int ball_dist_pix_g=pow(pix_x_g-real_pix_x_g,2)+pow(pix_y_g-real_pix_y_g,2);
        if(ball_dist_pix_g < pow(radius_sum_g,2)) {
          if (radius_g[i]>real_ball_radius_pix_g[j]) carpos_g[j][2]=-1;
          else diff_ball_g=false;
        }
      }
      if (diff_ball_g) {
        Scalar color = Scalar( 255, 0, 0);
        vector<float> ball_position_g;
        radius_g[i]=radius_g[i]-radcon;
        ball_position_g = pixel2point(center_g[i], radius_g[i]);
        float isx = ball_position_g[0];
        float isy = ball_position_g[1];
        float isz = ball_position_g[2];
        carpos_g[real_ball_g][0] = isx;
        carpos_g[real_ball_g][1] = isy;
        carpos_g[real_ball_g][2] = isz;
        real_ball_radius_pix_g[real_ball_g]=radius_g[i];
        real_ball_pix_g_x[real_ball_g]=center_g[i].x;
        real_ball_pix_g_y[real_ball_g]=center_g[i].y;
        real_ball_g++;
      }
    }
  }

  int small_ball_num_r=0;
  for (int i=0 ; i<real_ball_r;i++) {
    if (carpos_r[i][2]==-1) {
      small_ball_num_r++;
      continue;
    }
    for (int j=0; j<3 ; j++) carpos_r[i-small_ball_num_r][j]=carpos_r[i][j];
    real_ball_radius_pix_r[i-small_ball_num_r] = real_ball_radius_pix_r[i];
    real_ball_pix_r_x[i-small_ball_num_r] = real_ball_pix_r_x[i];
    real_ball_pix_r_y[i-small_ball_num_r] = real_ball_pix_r_y[i];
  } real_ball_r=real_ball_r-small_ball_num_r;

  int small_ball_num_b=0;
  for (int i=0 ; i<real_ball_b;i++) {
    if (carpos_b[i][2]==-1) {
      small_ball_num_b++;
      continue;
    }
    for (int j=0; j<3 ; j++) carpos_b[i-small_ball_num_b][j]=carpos_b[i][j];
    real_ball_radius_pix_b[i-small_ball_num_b] = real_ball_radius_pix_b[i];
    real_ball_pix_b_x[i-small_ball_num_b] = real_ball_pix_b_x[i];
    real_ball_pix_b_y[i-small_ball_num_b] = real_ball_pix_b_y[i];
  } real_ball_b=real_ball_b-small_ball_num_b;

  int small_ball_num_g=0;
  for (int i=0 ; i<real_ball_g;i++) {
    if (carpos_g[i][2]==-1) {
      small_ball_num_g++;
      continue;
    }
    for (int j=0; j<3 ; j++) carpos_g[i-small_ball_num_g][j]=carpos_g[i][j];
    real_ball_radius_pix_g[i-small_ball_num_g] = real_ball_radius_pix_g[i];
    real_ball_pix_g_x[i-small_ball_num_g] = real_ball_pix_g_x[i];
    real_ball_pix_g_y[i-small_ball_num_g] = real_ball_pix_g_y[i];
  } real_ball_g=real_ball_g-small_ball_num_g;


  Mat depth_frame;
  cv::resize(depth_mat_glo, depth_frame, cv::Size(1920, 1080));
  float* ball_mean_depth_r = new float[real_ball_r];
  for (int i=0 ; i<real_ball_r; i++) {
    Scalar color = Scalar(0,0,255);
    Mat find_depth_img_r(Size(1920, 1080), CV_8UC3, Scalar(0,0,0));
    float pix_r_x_rat = real_ball_pix_r_x[i];
    float pix_r_y_rat = real_ball_pix_r_y[i];
    float ball_rad_r_rat = real_ball_radius_pix_r[i];
    circle(find_depth_img_r, Point2f((int)pix_r_x_rat,(int)pix_r_y_rat),(int)ball_rad_r_rat,color,CV_FILLED,8,0);
    Mat mask_r;
    inRange(find_depth_img_r, Scalar(0, 0, 254, 0), Scalar(0, 0, 255, 0), mask_r);
    Scalar mean_value_r;
    mean_value_r = mean(depth_frame, mask_r); //Mean distance value of red pixels
    float mean_val_r = sum(mean_value_r)[0];
    ball_mean_depth_r[i]=mean_val_r;
  }
  float* ball_mean_depth_b = new float[real_ball_b];
  for (int i=0 ; i<real_ball_b; i++) {
    Scalar color = Scalar(255,0,0);
    float pix_b_x_rat = real_ball_pix_b_x[i];
    float pix_b_y_rat = real_ball_pix_b_y[i];
    float ball_rad_b_rat = real_ball_radius_pix_b[i];
    Mat find_depth_img_b(Size(1920, 1080), CV_8UC3, Scalar(0,0,0));
    circle(find_depth_img_b, Point2f((int)pix_b_x_rat,(int)pix_b_y_rat),(int)ball_rad_b_rat,color,CV_FILLED,8,0);
    Mat mask_b;
    inRange(find_depth_img_b, Scalar(254, 0, 0, 0), Scalar(255, 0, 0, 0), mask_b);
    Scalar mean_value_b;
    mean_value_b = mean(depth_frame, mask_b);		//Mean distance value of red pixels
    float mean_val_b = sum(mean_value_b)[0];
    ball_mean_depth_b[i]=mean_val_b;
  }
  float* ball_mean_depth_g = new float[real_ball_g];
  for (int i=0 ; i<real_ball_g; i++) {
    Scalar color = Scalar(0,255,0);
    float pix_g_x_rat = real_ball_pix_g_x[i];
    float pix_g_y_rat = real_ball_pix_g_y[i];
    float ball_rad_g_rat = real_ball_radius_pix_g[i];
    Mat find_depth_img_g(Size(1920, 1080), CV_8UC3, Scalar(0,0,0));
    circle(find_depth_img_g, Point2f((int)pix_g_x_rat,(int)pix_g_y_rat),(int)ball_rad_g_rat,color,CV_FILLED,8,0);
    Mat mask_g;
    inRange(find_depth_img_g, Scalar(0, 254, 0, 0), Scalar(0, 255, 0, 0), mask_g);
    Scalar mean_value_g;
    mean_value_g = mean(depth_frame, mask_g);		//Mean distance value of red pixels
    float mean_val_g = sum(mean_value_g)[0];
    ball_mean_depth_g[i]=mean_val_g;
  }


  for (int i=0 ; i<real_ball_r; i++) {
    Scalar color = Scalar( 0, 0, 255);
    float isx = carpos_r[i][0];
    float isy = carpos_r[i][1];
    float isz = carpos_r[i][2];
    string sx = floatToString(isx);
    string sy = floatToString(isy);
    string sz = floatToString(isz);
    text = "Red ball:" + sx + "," + sy + "," + sz;
    putText(result, text, Point2f(real_ball_pix_r_x[i],real_ball_pix_r_y[i]),2,1,Scalar(0,255,0),2);
    circle(result, Point2f(real_ball_pix_r_x[i],real_ball_pix_r_y[i]), (int)real_ball_radius_pix_r[i], color, 2, 8, 0 );
  }
  for (int i=0 ; i<real_ball_b ; i++) {
    Scalar color = Scalar( 255, 0, 0);
    float isx = carpos_b[i][0];
    float isy = carpos_b[i][1];
    float isz = carpos_b[i][2];
    string sx = floatToString(isx);
    string sy = floatToString(isy);
    string sz = floatToString(isz);
    text = "Blue ball:" + sx + "," + sy + "," + sz;
    putText(result, text, Point2f(real_ball_pix_b_x[i],real_ball_pix_b_y[i]),2,1,Scalar(0,255,0),2);
    circle( result, Point2f(real_ball_pix_b_x[i],real_ball_pix_b_y[i]), (int)real_ball_radius_pix_b[i], color, 2, 8, 0 );
  }

  for (int i=0 ; i<real_ball_g ; i++) {
    Scalar color = Scalar( 0, 255, 0);
    float isx = carpos_g[i][0];
    float isy = carpos_g[i][1];
    float isz = carpos_g[i][2];
    string sx = floatToString(isx);
    string sy = floatToString(isy);
    string sz = floatToString(isz);
    text = "Green ball:" + sx + "," + sy + "," + sz;
    putText(result, text, Point2f(real_ball_pix_g_x[i],real_ball_pix_g_y[i]),2,1,Scalar(0,255,0),2);
    circle( result, Point2f(real_ball_pix_g_x[i],real_ball_pix_g_y[i]), (int)real_ball_radius_pix_g[i], color, 2, 8, 0 );
  }


  core_msgs::ball_position msg;
  msg.r_size = real_ball_r;
  msg.r_img_x.resize(real_ball_r);
  msg.r_img_y.resize(real_ball_r);
  msg.b_size = real_ball_b;
  msg.b_img_x.resize(real_ball_b);
  msg.b_img_y.resize(real_ball_b);
  msg.g_size = real_ball_g;
  msg.g_img_x.resize(real_ball_g);
  msg.g_img_y.resize(real_ball_g);
  for(int k=0; k<real_ball_r; k++) {
    msg.r_img_x[k] = carpos_r[k][0]*100;
    msg.r_img_y[k] = (carpos_r[k][2]*cos(theta* PI / 180.0)-carpos_r[k][1]*sin(theta* PI / 180.0))*100;
    std::cout << ball_mean_depth_r[k] << endl;
  }
  for(int k=0; k<real_ball_b; k++) {
    msg.b_img_x[k] = carpos_b[k][0]*100;
    msg.b_img_y[k] = (carpos_b[k][2]*cos(theta* PI / 180.0)-carpos_b[k][1]*sin(theta* PI / 180.0))*100;
    std::cout << ball_mean_depth_b[k] << endl;
  }
  for(int k=0; k<real_ball_g; k++) {
    msg.g_img_x[k] = carpos_g[k][0]*100;
    msg.g_img_y[k] = (carpos_g[k][2]*cos(theta* PI / 180.0)-carpos_g[k][1]*sin(theta* PI / 180.0))*100;
    std::cout << ball_mean_depth_g[k] << endl;
  }


  namedWindow("result", WINDOW_NORMAL);
  imshow("result",result);
  waitKey(50);
  visualization_msgs::Marker ball_list;  //declare marker
  ball_list.header.frame_id = "/camera_link";  //set the frame
  ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.os
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
  pub.publish(msg);  //publish a message
  if (real_ball_r>0 && contours_r.size()>0) {
    delete real_ball_radius_pix_r;
    delete real_ball_pix_r_x;
    delete real_ball_pix_r_y;
    delete carpos_r;
    delete ball_mean_depth_r;
  }
  if (real_ball_b>0 && contours_b.size()>0) {
    delete real_ball_radius_pix_b;
    delete real_ball_pix_b_x;
    delete real_ball_pix_b_y;
    delete carpos_b;
    delete ball_mean_depth_b;
  }
  if (real_ball_g>0 && contours_g.size()>0) {
    delete real_ball_radius_pix_g;
    delete real_ball_pix_g_x;
    delete real_ball_pix_g_y;
    delete carpos_g;
    delete ball_mean_depth_g;
  }
  */
}
void exitCallback(const std_msgs::Int32::ConstPtr& msg){
  ROS_INFO("exit message : %d",msg->data);
	if (msg->data > 5){
		ROS_INFO("ball_detect node process terminated");
		exit(0);
	}
}

void depthImage_cb(const sensor_msgs::ImageConstPtr &msg_depth){
  try {
  	cv_bridge::CvImageConstPtr depth_img_cv;
  	depth_img_cv = cv_bridge::toCvShare (msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
  	depth_img_cv->image.convertTo(depth_mat_glo, CV_32F, 0.001);
    ball_detect();
    //cv::resize(depth_mat_glo,depth_mat_glo,cv::Size(1280,720));
    waitKey(1);
  }
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'z16'.", msg_depth->encoding.c_str());}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "ball_detect_node"); //init ros nodd
  ros::NodeHandle nh; //create node handler
  ros::Subscriber sub3 = nh.subscribe<std_msgs::Int32>("/picked_balls",1,exitCallback);
  image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
  image_transport::Subscriber sub_depth = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthImage_cb);
  image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);
  pub = nh.advertise<core_msgs::ball_position>("/position", 1); //setting publisher
  ros::Rate loop_rate(30);
  while(ros::ok()){
     ros::spinOnce();
     loop_rate.sleep();
  }


  return 0;
}
