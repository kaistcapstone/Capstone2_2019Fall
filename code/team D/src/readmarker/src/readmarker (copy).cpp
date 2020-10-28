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


using namespace cv;
using namespace std;

Mat inputImage;
std::vector<int> markerIds;
std::vector<std::vector<Point2f> > markerCorners, rejectedCandidates;
std::vector<Point2f> innerMarkerCorners;
std::vector<Point2f> outImageCorners;

Ptr<aruco::DetectorParameters> parameters;
Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

Mat image_1;
Mat image_2;

int n_image1;
int n_image2;

ros::Publisher input_status_pub;

sensor_msgs::ImagePtr pub_msg1;
sensor_msgs::ImagePtr pub_msg2;

core_msgs::markermsg cropped_img_msg;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
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

  if (markerIds.size() > 0)
  {
      aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

      //Check presence

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

      if(n_image1==1)
      {
        outImageCorners.resize(4);
        innerMarkerCorners.resize(4);
        outImageCorners[0] = cvPoint(0.0, 0.0);
        innerMarkerCorners[0] = markerCorners[indice[0]][2];
        outImageCorners[1] = cvPoint(255.0, 0.0);
        innerMarkerCorners[1] = markerCorners[indice[1]][3];
        outImageCorners[2] = cvPoint(255.0, 255.0);
        innerMarkerCorners[2] = markerCorners[indice[2]][0];
        outImageCorners[3] = cvPoint(0.0, 255.0);
        innerMarkerCorners[3] = markerCorners[indice[3]][1];
        Mat H1 = findHomography(innerMarkerCorners,outImageCorners,0);
        warpPerspective(inputImage, image_1, H1, cv::Size(255, 255));

      }
      if(n_image2==1)
      {
        outImageCorners.resize(4);
        innerMarkerCorners.resize(4);
        outImageCorners[0] = cvPoint(0.0, 0.0);
        innerMarkerCorners[0] = markerCorners[indice[4]][2];
        outImageCorners[1] = cvPoint(255.0, 0.0);
        innerMarkerCorners[1] = markerCorners[indice[5]][3];
        outImageCorners[2] = cvPoint(255.0, 255.0);
        innerMarkerCorners[2] = markerCorners[indice[6]][0];
        outImageCorners[3] = cvPoint(0.0, 255.0);
        innerMarkerCorners[3] = markerCorners[indice[7]][1];
        Mat H2 = findHomography(innerMarkerCorners,outImageCorners,0);
        warpPerspective(inputImage, image_2, H2, cv::Size(255, 255));
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

  Mat showImg = Mat::zeros(Size(256,512),CV_8UC3);


  image_1.copyTo(showImg(Rect(0,0,image_1.cols, image_1.rows)));
  image_2.copyTo(showImg(Rect(0,image_1.rows,image_2.cols,image_2.rows)));


  imshow("image", showImg);
  imshow("out", outputImage);

  moveWindow("image", outputImage.cols+50,20);
  moveWindow("out", 50,20);


  cvWaitKey(20);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_reader");
  ros::NodeHandle nh;
  input_status_pub = nh.advertise<core_msgs::markermsg>("cropped_img", 100);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::Rate loop_rate(5);
  ros::spin();
}
