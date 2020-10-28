//Reading AruCo markers
//Author : Dongha Chung
//Reference : https://docs.opencv.org/3.3.0/d5/dae/tutorial_aruco_detection.html

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <std_msgs/Int8.h>
// #include "readmarker/markermsg.h"
#include "core_msgs/markermsg.h"
#include "core_msgs/markerposition.h"

using namespace cv;
using namespace std;

Mat inputImage;
vector<int> markerIds;
vector<vector<Point2f> > markerCorners, rejectedCandidates;
vector<Point2f> innerMarkerCorners;
vector<Point2f> outImageCorners;

Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
Mat image_1;
Mat image_2;

int b_image1;
int b_image2;

ros::Publisher input_status_pub;
ros::Publisher marker_corner_pub;

core_msgs::markermsg cropped_img_msg;
core_msgs::markerposition marker_msg;
void Initialization()
{
  outImageCorners.resize(4);
  outImageCorners[0] = cvPoint(0.0, 0.0);
  outImageCorners[1] = cvPoint(255.0, 0.0);
  outImageCorners[2] = cvPoint(255.0, 255.0);
  outImageCorners[3] = cvPoint(0.0, 255.0);
  b_image1 = 1;
  b_image2 = 1;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  b_image1=1;
  b_image2=1;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_ptr->image.copyTo(inputImage);

  aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

  Mat outputImage;
  inputImage.copyTo(outputImage);
  image_1 = Mat(256,256, CV_8UC3, Scalar(255,255,255));
  image_2 = Mat(256,256, CV_8UC3, Scalar(255,255,255));

  if (markerIds.size() > 0)
  {
      aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

      vector<int> indice;
      indice.resize(8);
      // printf("%d",*markerIds(1));


      for(int i=0;i<4;i++)
      {
        if(find(markerIds.begin(), markerIds.end(), i+1)!=markerIds.end())
        {
          indice[i]= distance(markerIds.begin(), find(markerIds.begin(), markerIds.end(), i+1));
        }
        else{

          // if(i==0 || i==1){
          b_image1 = b_image1*0;
          // }
            }


      }

      for(int i=4;i<8;i++)
      {
        if(find(markerIds.begin(), markerIds.end(), i+1)!=markerIds.end())
        {
          b_image2=1;
          indice[i]= distance(markerIds.begin(), find(markerIds.begin(), markerIds.end(), i+1));
          printf("index =%d, value =  %d\n", i,indice[i]);
        }
        else{
        //   if(i==4 || i==5){
          b_image2 = b_image2*0;
        // }

        }


      }



      if(b_image1==1)
      {

        innerMarkerCorners.resize(4);
        innerMarkerCorners[0] = markerCorners[indice[0]][2];
        innerMarkerCorners[1] = markerCorners[indice[1]][3];
        innerMarkerCorners[2] = markerCorners[indice[2]][0];
        innerMarkerCorners[3] = markerCorners[indice[3]][1];
        Mat H1 = findHomography(innerMarkerCorners,outImageCorners,0);
        marker_msg.x1=innerMarkerCorners[0].x;
        marker_msg.y1=innerMarkerCorners[0].y;
        marker_msg.x2=innerMarkerCorners[1].x;
        marker_msg.y2=innerMarkerCorners[1].y;
        marker_msg.x3=innerMarkerCorners[2].x;
        marker_msg.y3=innerMarkerCorners[2].y;
        marker_msg.x4=innerMarkerCorners[3].x;
        marker_msg.y4=innerMarkerCorners[3].y;
        marker_corner_pub.publish(marker_msg);
        warpPerspective(inputImage, image_1, H1, Size(255, 255));

      }

      if(b_image2==1)
      {
        printf("b_image2==1\n");
        innerMarkerCorners.resize(4);
        innerMarkerCorners[0] = markerCorners[indice[4]][2];
        innerMarkerCorners[1] = markerCorners[indice[5]][3];
        innerMarkerCorners[2] = markerCorners[indice[6]][0];
        innerMarkerCorners[3] = markerCorners[indice[7]][1];
        cout<<innerMarkerCorners[0]<<endl;
        cout<<innerMarkerCorners[1]<<endl;
        cout<<innerMarkerCorners[2]<<endl;
        cout<<innerMarkerCorners[3]<<endl;

        marker_msg.x1=innerMarkerCorners[0].x;
        marker_msg.y1=innerMarkerCorners[0].y;
        marker_msg.x2=innerMarkerCorners[1].x;
        marker_msg.y2=innerMarkerCorners[1].y;
        marker_msg.x3=innerMarkerCorners[2].x;
        marker_msg.y3=innerMarkerCorners[2].y;
        marker_msg.x4=innerMarkerCorners[3].x;
        marker_msg.y4=innerMarkerCorners[3].y;
        marker_corner_pub.publish(marker_msg);
        Mat H2 = findHomography(innerMarkerCorners,outImageCorners,0);
        warpPerspective(inputImage, image_2, H2, Size(255, 255));
      }

  }
  else
  {
    b_image1 = 0;
    b_image2 = 0;
  }

  cropped_img_msg.image1_available = b_image1;
  cropped_img_msg.image2_available = b_image2;

  imencode(".jpg", image_1, cropped_img_msg.cimage1.data);
  imencode(".jpg", image_2, cropped_img_msg.cimage2.data);
  input_status_pub.publish(cropped_img_msg);
  Mat showImg = Mat::zeros(Size(256,512),CV_8UC3);
  image_1.copyTo(showImg(Rect(0,0,image_1.cols, image_1.rows)));
  image_2.copyTo(showImg(Rect(0,image_1.rows,image_2.cols,image_2.rows)));

  imshow("image", showImg);
  imshow("out", outputImage);

  moveWindow("image", outputImage.cols+50,20);
  moveWindow("out", 50,20);
  cvWaitKey(1);
}

int main(int argc, char** argv)
{
  Initialization();
  ros::init(argc, argv, "marker_reader");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  input_status_pub = nh.advertise<core_msgs::markermsg>("cropped_img", 100);
  marker_corner_pub = nh.advertise<core_msgs::markerposition>("marker_pixel_cnn", 100);
  // ros::Publisher pub3 = nh.advertise<std_msgs::Int32>("STOP_motion", 1000);
  // input_status_pub = nh.advertise<core_msgs::markermsg>("cropped_img", 1);
  image_transport::ImageTransport it(nh); //Set the image transport it using the node handle nh
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  while(ros::ok()){
    ros::spinOnce();
    // printf("sldfiwejlif");
    loop_rate.sleep();
  }
}
