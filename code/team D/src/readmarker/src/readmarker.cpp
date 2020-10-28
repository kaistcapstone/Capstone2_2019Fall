#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <algorithm>
#include <std_msgs/Int8.h>
#include "core_msgs/markermsg.h"
#include "core_msgs/mark.h"

using namespace cv;
using namespace std;

Mat inputImage;
Mat buffer2(320,240,CV_16UC1);
std::vector<int> markerIds;
std::vector<std::vector<Point2f> > markerCorners, rejectedCandidates;
std::vector<Point2f> innerMarkerCorners;
std::vector<Point2f> outImageCorners;

float intrinsic_data[9] = {614.9002685546875, 0.0, 324.05169677734375, 0.0, 615.0999145507812, 236.6910858154297, 0.0, 0.0, 1.0};
float distortion_data[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
void calibrate(Mat &frame);


Ptr<aruco::DetectorParameters> parameters;
Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

Mat image_1;
Mat image_2;

int n_image1;
int n_image2;

ros::Publisher input_status_pub;
ros::Publisher input_status_pub2;

sensor_msgs::ImagePtr pub_msg1;
sensor_msgs::ImagePtr pub_msg2;

core_msgs::markermsg cropped_img_msg;


vector<float> pixel2point(Point center, int distance);
void calculate(Point center,Point outpoint, int distance);
int height = (340-225);
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  core_msgs::mark msg_marker;

  n_image1=1;
  n_image2=1;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_ptr->image.copyTo(inputImage);

  aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

  Mat outputImage;
  inputImage.copyTo(outputImage);
  image_1 = Mat(256,256, CV_8UC3, Scalar(255,255,255));
  image_2 = Mat(256,256, CV_8UC3, Scalar(255,255,255));
  calibrate(outputImage);
  if (markerIds.size() > 0)
  {
      aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

      //Check presence
//cout<<markerCorners<<endl;
      std::vector<int> indice;
      indice.resize(8);

      for(int i=0;i<4;i++)
      {
        if(std::find(markerIds.begin(), markerIds.end(), i+1)!=markerIds.end()){
          indice[i]= std::distance(markerIds.begin(), std::find(markerIds.begin(), markerIds.end(), i+1));

        }
        else
          n_image1 = n_image1*0;
      }
      for(int i=4;i<8;i++)
      {
        if(std::find(markerIds.begin(), markerIds.end(), i+1)!=markerIds.end()){
          indice[i]= std::distance(markerIds.begin(), std::find(markerIds.begin(), markerIds.end(), i+1));

        }
        else
          n_image2 = n_image2*0;
      }
      int x,y, x1,y1;
      Point mark(x,y),mark1(x1,y1);
      int distance1, distance2;
      if(n_image1==1)
      {
        outImageCorners.resize(4);
        innerMarkerCorners.resize(4);
        outImageCorners[0] = cvPoint(0.0, 0.0);
        innerMarkerCorners[0] = markerCorners[indice[0]][2];
	int xone;
        int yone;
        int distanceone;
        int xtwo;
        int ytwo;
        int distancetwo;
        int Xc;
        int Yc;
        int Zc;
        xone=markerCorners[indice[0]][2].x;
        yone=markerCorners[indice[0]][2].y;
        distanceone = buffer2.at<ushort>(yone,xone);
        Zc=distanceone;
        Xc = (xone-intrinsic_data[2])/intrinsic_data[0]*Zc;
        Yc = (yone-intrinsic_data[5])/intrinsic_data[4]*Zc;
        distanceone = sqrt(pow(Zc,2)-pow(Xc,2)-pow(height,2));
        cout<<"Xc="<<Xc<<"\t"<<"Yc="<<Yc<<"\t"<<Zc<<endl;
        msg_marker.x1 = Xc;
        msg_marker.y1 = distanceone;
        msg_marker.z1 = Zc;
        outImageCorners[1] = cvPoint(255.0, 0.0);
        innerMarkerCorners[1] = markerCorners[indice[1]][3];
        xtwo=markerCorners[indice[1]][3].x;
        ytwo=markerCorners[indice[1]][3].y;
        distancetwo = buffer2.at<ushort>(ytwo,xtwo);
        Xc = (xtwo-intrinsic_data[2])/intrinsic_data[0]*Zc;
        Yc = (ytwo-intrinsic_data[5])/intrinsic_data[4]*Zc;
        Zc=distancetwo;
        distancetwo = sqrt(pow(Zc,2)-pow(Xc,2)-pow(height,2));
        msg_marker.x2 = Xc;
        msg_marker.y2 = distancetwo;
        msg_marker.z2 = Zc;
        cout<<"Xc2="<<Xc<<"\t"<<"Yc2="<<Yc<<"\t"<<Zc<<endl;
        outImageCorners[2] = cvPoint(255.0, 255.0);
        innerMarkerCorners[2] = markerCorners[indice[2]][0];
        outImageCorners[3] = cvPoint(0.0, 255.0);
        innerMarkerCorners[3] = markerCorners[indice[3]][1];
        Mat H1 = findHomography(innerMarkerCorners,outImageCorners,0);
        warpPerspective(inputImage, image_1, H1, cv::Size(255, 255));
        //input_status_pub2.publish(msg_marker);


      }
      if(n_image2==1)
      {
        outImageCorners.resize(4);
        innerMarkerCorners.resize(4);
        outImageCorners[0] = cvPoint(0.0, 0.0);
        innerMarkerCorners[0] = markerCorners[indice[4]][2];
        // calculate(markerCorners[indice[4]][2],mark,distance1);
        // msg_marker.x1 =  mark.x;
        // msg_marker.y1 = mark.y;
        // msg_marker.z1 = distance1;
        int xone;
        int yone;
        int distanceone;
        int xtwo;
        int ytwo;
        int distancetwo;
        int Xc;
        int Yc;
        int Zc;
        xone=markerCorners[indice[4]][2].x;
        yone=markerCorners[indice[4]][2].y;
        distanceone = buffer2.at<ushort>(yone,xone);
        Zc=distanceone;
        Xc = (xone-intrinsic_data[2])/intrinsic_data[0]*Zc;
        Yc = (yone-intrinsic_data[5])/intrinsic_data[4]*Zc;
	distanceone = sqrt(pow(Zc,2)-pow(Xc,2)-pow(height,2));
        cout<<"Xc="<<Xc<<"\t"<<"Yc="<<Yc<<"\t"<<"d="<<Zc<<"\t"<<"z="<<distanceone<<endl;
        msg_marker.x1 = Xc;
        msg_marker.y1 = distanceone;
        msg_marker.z1 = Zc;
        outImageCorners[1] = cvPoint(255.0, 0.0);
        innerMarkerCorners[1] = markerCorners[indice[5]][3];
        // calculate(markerCorners[indice[5]][3],mark,distance2);
        // msg_marker.x2 = mark1.x;
        // msg_marker.y2 = mark1.y;
        // msg_marker.z2 = distance2;
        xtwo=markerCorners[indice[5]][3].x;
        ytwo=markerCorners[indice[5]][3].y;
        distancetwo = buffer2.at<ushort>(ytwo,xtwo);
        Xc = (xtwo-intrinsic_data[2])/intrinsic_data[0]*Zc;
        Yc = (ytwo-intrinsic_data[5])/intrinsic_data[4]*Zc;
        Zc=distancetwo;
        distancetwo = sqrt(pow(Zc,2)-pow(Xc,2)-pow(height,2));
        msg_marker.x2 = Xc;
        msg_marker.y2 = distancetwo;
        msg_marker.z2 = Zc;

        cout<<"Xc2="<<Xc<<"\t"<<"Yc2="<<Yc<<"\t"<<"d"<<Zc<<"\t"<<"z="<<distancetwo<<endl;
        outImageCorners[2] = cvPoint(255.0, 255.0);
        innerMarkerCorners[2] = markerCorners[indice[6]][0];
        outImageCorners[3] = cvPoint(0.0, 255.0);
        innerMarkerCorners[3] = markerCorners[indice[7]][1];
        Mat H2 = findHomography(innerMarkerCorners,outImageCorners,0);
        warpPerspective(inputImage, image_2, H2, cv::Size(255, 255));
//        input_status_pub2.publish(msg_marker);

      }
  }
  else
  {
    n_image1 = 0;
    n_image2 = 0;
  }
  pub_msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_1).toImageMsg();
  pub_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_2).toImageMsg();
  cropped_img_msg.image1_available = n_image1;
  cropped_img_msg.image2_available = n_image2;
  cropped_img_msg.image1 = *pub_msg1;
  cropped_img_msg.image2 = *pub_msg2;
  imencode(".jpg", image_1, cropped_img_msg.cimage1.data);
  imencode(".jpg", image_2, cropped_img_msg.cimage2.data);
  input_status_pub.publish(cropped_img_msg);
  input_status_pub2.publish(msg_marker);

  Mat showImg = Mat::zeros(Size(256,512),CV_8UC3);


  image_1.copyTo(showImg(Rect(0,0,image_1.cols, image_1.rows)));
  image_2.copyTo(showImg(Rect(0,image_1.rows,image_2.cols,image_2.rows)));


  imshow("image", showImg);
  imshow("out", outputImage);

  moveWindow("image", outputImage.cols+50,20);
  moveWindow("out", 50,20);


  cvWaitKey(20);

}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
   //ROS_INFO("Depth image height = %d, width = %d",msg->height, msg->width);
   cv_bridge::CvImageConstPtr cv_ptrD;
   if(msg->height==480&&buffer2.size().width==320){  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_reader");
  ros::NodeHandle nh;
  input_status_pub = nh.advertise<core_msgs::markermsg>("cropped_img", 100);
  input_status_pub2 = nh.advertise<core_msgs::mark>("mark", 1);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub2 = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthCallback);
  image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback); //create

  ros::Rate loop_rate(5);
  ros::spin();
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

vector<float> pixel2point(Point center, int distance){
    float x, y, u, v, Xc, Yc,Zc;
    x = center.x;//.x;// .at(0);
    y = center.y;//.y;//
    Zc = distance;
    Xc = (x-intrinsic_data[2])/intrinsic_data[0]*Zc;
    Yc = (y-intrinsic_data[5])/intrinsic_data[4]*Zc;
    Xc = roundf(Xc * 1000) / 1000;
    Yc = roundf(Yc * 1000) / 1000;
    Zc = roundf(Zc * 1000) / 1000;
    center.x=Xc;
    center.y=Yc;
    cout<<"[ixel2point"<<endl;
}

void calculate(Point center,Point outpoint, int distance){
  int x,y;
  x=center.x;
  y=center.y;
  distance = buffer2.at<ushort>(y,x);
  outpoint = center;
  cout<<"calculate???"<<endl;
  pixel2point(outpoint, distance);
  cout<<"allpassed??"<<endl;

}
