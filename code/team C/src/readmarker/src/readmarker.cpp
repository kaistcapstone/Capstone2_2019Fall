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
#include "core_msgs/wall_distance.h"


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

Mat depth_mat_glo;

ros::Publisher input_status_pub;
ros::Publisher input_status_pub_dist;


sensor_msgs::ImagePtr pub_msg1;
sensor_msgs::ImagePtr pub_msg2;

core_msgs::markermsg cropped_img_msg;
core_msgs::wall_distance wall_distance_msg;


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


Mat make_mask(vector<Point> marker_point_in)
{
  Mat find_depth_raw=Mat::zeros(720, 1280, CV_8UC1);
  Mat find_depth_img;
  const Point *pts = (const cv::Point*) Mat(marker_point_in).data;
	int npt = Mat(marker_point_in).rows;
  fillPoly(find_depth_raw, &pts, &npt, 1, 255);
  inRange(find_depth_raw, 254, 255, find_depth_img);
  return find_depth_img;
}
// int true_count(bool marker[8]){
//   int count = 0;
//   for(int i = 0; i<8; i++){
//     if(marker[i]){
//       count++;
//     }
//   }
//   return count;
// }
vector<float> find_marker_depth(vector<vector<Point> > marker_point)
{
  vector<float> distance_in;
  Mat depth_frame = Mat::zeros(1280, 720, CV_32F);
  depth_frame = depth_mat_glo;
  // if (true_count(marker_exist) > 0)
  // {
  //   for (int i=0;i<true_count(marker_exist);i++)
  //   {
  //     Mat inrange_img=make_mask(marker_point[i]);
  //     Scalar mean_value;
  //     mean_value = mean(depth_frame, inrange_img); //Mean distance value of red pixels
  //     float mean_val_raw = sum(mean_value)[0];
  //     float mean_val = roundf(mean_val_raw * 1000) / 1000;
  //     distance_in.push_back(mean_val);
  //   }
  //
  //   for(int i = true_count(marker_exist)-1; i < 4; i++ ){
  //     distance_in.push_back(0.0);
  //   }
  // }

  if (not(depth_frame.empty()))
  {
    for (int i=0;i<4;i++)
    {
      Mat inrange_img=make_mask(marker_point[i]);
      Scalar mean_value;
      mean_value = mean(depth_frame, inrange_img); //Mean distance value of red pixels
      float mean_val_raw = sum(mean_value)[0];
      float mean_val = roundf(mean_val_raw * 1000) / 1000;
      distance_in.push_back(mean_val);
    }
  }

  else
  {
    for (int i=0;i<4;i++)
    {
      distance_in.push_back(0.0);
    }
  }
  for(int i = 0; i<distance_in.size(); i++){
    cout<<"marker vec: "<<distance_in[i]<<endl;

  }

  return distance_in;
}

void draw_marker_depth(Mat base_img, vector<Point2f> marker_point, vector<float> marker_dis)
{
  for (int i=0;i<4;i++)
  {
    if (marker_dis[i]>0)
    {
      int isx=marker_point[i].x;
      int isy=marker_point[i].y;
      float isz = marker_dis[i];
      string sx = intToString(isx);
      string sy = intToString(isy);
      string sz = floatToString(isz);
      string text = "("+ sx +", "+ sz + ")";
      putText(base_img, text, marker_point[i],2,0.4,Scalar(0,255,0),1);
    }
  }
}



void depthImage_cb(const sensor_msgs::ImageConstPtr &msg_depth){
  try {
  	cv_bridge::CvImageConstPtr depth_img_cv;
  	depth_img_cv = cv_bridge::toCvShare (msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
  	depth_img_cv->image.convertTo(depth_mat_glo, CV_32F, 0.001);
    //cv::resize(depth_mat_glo,depth_mat_glo,cv::Size(1280,720));
    waitKey(1);
  }
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'z16'.", msg_depth->encoding.c_str());}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  n_image1=1;
  n_image2=1;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_ptr->image.copyTo(inputImage);

  aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

  Mat outputImage;
  Mat outputImage_depth;
  inputImage.copyTo(outputImage);
  inputImage.copyTo(outputImage_depth);
  image_1 = Mat(256,256, CV_8UC3, Scalar(255,255,255));
  image_2 = Mat(256,256, CV_8UC3, Scalar(255,255,255));
  bool marker_exist_tot = false;
  bool marker_exist[8] = {false,false,false,false,false,false,false,false};
  vector<vector<Point> > marker_dis_1;
  vector<vector<Point> > marker_dis_2;
  vector<float> distance_mar_1;
  vector<float> distance_mar_2;
  vector<Point2f> innerMarkerCorners_1;
  innerMarkerCorners_1.resize(4);
  std::vector<int> indice;
  indice.resize(8);
  outImageCorners.resize(4);
  wall_distance_msg.marker_exist = 0;
  wall_distance_msg.marker_x.resize(8);
  wall_distance_msg.marker_distance.resize(8);

  for(int i=0;i<8;i++)
  {
    wall_distance_msg.marker_x[i]=0.0;
    wall_distance_msg.marker_distance[i]=0.0;
  }
  if (markerIds.size() > 0)
  {
      aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

      //Check presence

      marker_dis_1.resize(4);
      marker_dis_2.resize(4);


      for (int i=0; i<4;i++)
      {
        marker_dis_1[i].resize(4);
        marker_dis_2[i].resize(4);
      }



      for(int i=0;i<4;i++)
      {
        if(std::find(markerIds.begin(), markerIds.end(), i+1)!=markerIds.end()){
          indice[i]= std::distance(markerIds.begin(), std::find(markerIds.begin(), markerIds.end(), i+1));
          marker_exist_tot = true;
          marker_exist[i] = true;
          wall_distance_msg.marker_exist = 1;
        }
        else
          n_image1 = n_image1*0;
      }
      for(int i=4;i<8;i++)
      {
        if(std::find(markerIds.begin(), markerIds.end(), i+1)!=markerIds.end()){
          indice[i]= std::distance(markerIds.begin(), std::find(markerIds.begin(), markerIds.end(), i+1));
          marker_exist[i] = true;
          wall_distance_msg.marker_exist = 1;
        }
        else
          n_image2 = n_image2*0;
      }

      if(n_image1==1)
      {

        outImageCorners[0] = cvPoint(0.0, 0.0);
        outImageCorners[1] = cvPoint(255.0, 0.0);
        outImageCorners[2] = cvPoint(255.0, 255.0);
        outImageCorners[3] = cvPoint(0.0, 255.0);
        innerMarkerCorners_1[0] = markerCorners[indice[0]][2];
        innerMarkerCorners_1[1] = markerCorners[indice[1]][3];
        innerMarkerCorners_1[2] = markerCorners[indice[2]][0];
        innerMarkerCorners_1[3] = markerCorners[indice[3]][1];

        Mat H1 = findHomography(innerMarkerCorners_1,outImageCorners,0);
        warpPerspective(inputImage, image_1, H1, cv::Size(255, 255));

        distance_mar_1 = find_marker_depth(marker_dis_1);
        draw_marker_depth(outputImage, innerMarkerCorners_1, distance_mar_1);

        for (int i=0;i<4;i++)
        {
          wall_distance_msg.marker_x[i]=innerMarkerCorners_1[i].x;
          wall_distance_msg.marker_distance[i]=distance_mar_1[i];
        }
        wall_distance_msg.marker_exist = 2;

      }
      if(n_image2==1)
      {
        outImageCorners.resize(4);
        innerMarkerCorners.resize(4);
        vector<Point2f> innerMarkerCorners_2;
        innerMarkerCorners_2.resize(4);
        outImageCorners[0] = cvPoint(0.0, 0.0);
        innerMarkerCorners[0] = markerCorners[indice[4]][2];
        outImageCorners[1] = cvPoint(255.0, 0.0);
        innerMarkerCorners[1] = markerCorners[indice[5]][3];
        outImageCorners[2] = cvPoint(255.0, 255.0);
        innerMarkerCorners[2] = markerCorners[indice[6]][0];
        outImageCorners[3] = cvPoint(0.0, 255.0);
        innerMarkerCorners[3] = markerCorners[indice[7]][1];
        for (int i=4;i<8;i++)
        {
          for (int j=0;j<4;j++)
          {
            marker_dis_2[i-4][j]=markerCorners[indice[i]][j];
          }
          innerMarkerCorners_2[i-4]=innerMarkerCorners[i-4];
        }
        distance_mar_2 = find_marker_depth(marker_dis_2);
        draw_marker_depth(outputImage, innerMarkerCorners_2, distance_mar_2);
        Mat H2 = findHomography(innerMarkerCorners,outImageCorners,0);
        warpPerspective(inputImage, image_2, H2, cv::Size(255, 255));

        for (int i=4;i<8;i++)
        {
          wall_distance_msg.marker_x[i]=innerMarkerCorners_2[i-4].x;
          wall_distance_msg.marker_distance[i]=distance_mar_2[i-4];
        }
        wall_distance_msg.marker_exist = 2;
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
  if(n_image1 == 1 || n_image2 ==1)
  {
    input_status_pub.publish(cropped_img_msg);
  }
  for (int i=0;i<4;i++)
  {
    if (marker_exist[i])
    {
      for (int j=0;j<4;j++)
      {
        marker_dis_1[i][j]=markerCorners[indice[i]][j];
      //  cout<<"marker dis 1"<<"["<<i<<"]"<<"["<<j<<"]: "<<marker_dis_1[i][j]<<endl;
      }
      cout<<"checl_1"<<endl;


    }
  }

  if (marker_exist_tot)
  {
    distance_mar_1 = find_marker_depth(marker_dis_1);
    cout<<"checl_2"<<endl;
    draw_marker_depth(outputImage, innerMarkerCorners_1, distance_mar_1);
    cout<<"checl_3"<<endl;

    for (int i=0;i<4;i++)
    {
      cout<<"checl_4"<<endl;
      wall_distance_msg.marker_x[i]=innerMarkerCorners_1[i].x;
      if(innerMarkerCorners_1[i].x !=0)
      {
        wall_distance_msg.marker_distance[i]=distance_mar_1[i];
        cout<<"checl_5"<<endl;
      }
    }
  }
  input_status_pub_dist.publish(wall_distance_msg);

  Mat showImg = Mat::zeros(Size(256,512),CV_8UC3);


  image_1.copyTo(showImg(Rect(0,0,image_1.cols, image_1.rows)));
  image_2.copyTo(showImg(Rect(0,image_1.rows,image_2.cols,image_2.rows)));


  imshow("image", showImg);
  imshow("out", outputImage);

  //moveWindow("image", outputImage.cols+50,20);
  //moveWindow("out", 50,20);


  cvWaitKey(20);

}

int main(int argc, char** argv)
{
  namedWindow("image", WINDOW_NORMAL);
  namedWindow("out", WINDOW_NORMAL);
  //namedWindow("fillpoly", WINDOW_NORMAL);
  //namedWindow("depth", WINDOW_NORMAL);

  ros::init(argc, argv, "marker_reader");
  ros::NodeHandle nh;
  input_status_pub = nh.advertise<core_msgs::markermsg>("cropped_img", 1);
  input_status_pub_dist = nh.advertise<core_msgs::wall_distance>("marker_info", 1);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_depth = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthImage_cb);
  image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);


  ros::Rate loop_rate(5);
  ros::spin();
}
