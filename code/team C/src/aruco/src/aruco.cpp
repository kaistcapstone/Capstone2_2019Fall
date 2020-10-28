//Making AruCo markers
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

using namespace cv;
using namespace std;

int main()
{
  Mat markerImage;
  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
  string imgtitle_dir = "/home/dongha/Capstone_Programs/src/webcam/src/";
  string imgtitle_pre = "Marker No. ";
  string img_format = ".jpg";
  string imgtitle;
  for(int i=1;i<9;i++)
  {
    aruco::drawMarker(dictionary, i, 200, markerImage, 1);
    imgtitle = imgtitle_dir+imgtitle_pre+boost::lexical_cast<std::string>(i)+img_format;
    imwrite(imgtitle,markerImage);
  }
  return 0;
}
