#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include "visy_detector_pkg/DetectConveyorSystem.h"

static const std::string OPENCV_WINDOW1 = "Image window1";
static const std::string OPENCV_WINDOW2 = "Image window2";
static const std::string OPENCV_WINDOW3 = "Image window3";
static const std::string OPENCV_WINDOW4 = "Image window4";
static const std::string OPENCV_WINDOW5 = "Image window5";
using namespace cv;
using namespace std;

class ConveyorDetectorNode
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  cv::Mat imagework;
  cv::Mat imagesrc;
  cv::Mat imagegray;
  cv::Mat imagehsv;
  std::vector<cv::Mat> imagesplit;
  cv::Mat saturation, saturationf32;
  cv::Mat value, value32;
  cv::Mat chroma;
  int check = 0;
  vector<Point2f> beltpoints;
  uint counter = 0;
  chrono::time_point<std::chrono::system_clock> start, end, latenz_start, latenz_end;
  Point2f MC, MC_PRE;
  ros::ServiceServer detectConveyorSystemService;
  ros::Publisher conveyorSystemRectPub;
  std_msgs::Float32MultiArray conveyorSystemRect;

public:
  ConveyorDetectorNode(): it_(nh_)
  {
    detectConveyorSystemService = nh_.advertiseService("detect_conveyor_system", &ConveyorDetectorNode::detectConveyorSystemCB,this);
    conveyorSystemRectPub = nh_.advertise<std_msgs::Float32MultiArray>("conveyor_system_rect", 1);
  }
  ~ConveyorDetectorNode()
  {
  }

  bool detectConveyorSystemCB(visy_detector_pkg::DetectConveyorSystem::Request  &req, visy_detector_pkg::DetectConveyorSystem::Response &res)
  {
    cv::namedWindow(OPENCV_WINDOW1);
    cv::namedWindow(OPENCV_WINDOW2);
    cv::namedWindow(OPENCV_WINDOW3);
    cv::namedWindow(OPENCV_WINDOW4);
    cv::namedWindow(OPENCV_WINDOW5);
    image_sub_ = it_.subscribe("/raspicam_node/image", 1, &ConveyorDetectorNode::imageCb, this);
    counter = 0;

    while(counter<10)
    {
      ros::spinOnce();
    }
    image_sub_.shutdown();

    conveyorSystemRect.data.clear();

    for(auto const& p:beltpoints)
    {
      conveyorSystemRect.data.push_back(p.x);
      conveyorSystemRect.data.push_back(p.y);
      conveyorSystemRectPub.publish(conveyorSystemRect);
    }

    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
    cv::destroyWindow(OPENCV_WINDOW3);
    cv::destroyWindow(OPENCV_WINDOW4);
    cv::destroyWindow(OPENCV_WINDOW5);

    return true;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    imagework = cv_ptr->image.clone();
    imagesrc = cv_ptr->image.clone();

    ROS_INFO("%d", counter+1);

    if(counter<10)
    {
      cv::cvtColor(imagework, imagehsv, CV_BGR2HSV);

      split(imagehsv, imagesplit);

      saturation = imagesplit[1].clone();
      saturation.convertTo(saturationf32, CV_32FC1, 1. / 255);

      value = imagesplit[2].clone();
      value.convertTo(value32, CV_32FC1, 1. / 255);

      add(value32, saturationf32, chroma);

      chroma.convertTo(chroma, CV_8UC1, 200);

      imagework = chroma.clone();

      cv::imshow(OPENCV_WINDOW1, imagework);

      //cv::adaptiveThreshold(imagework,imagework,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,21,2);
      cv::adaptiveThreshold(imagework,imagework,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,9,2);

      cv::imshow(OPENCV_WINDOW2, imagework);

      //cv::Mat Element = getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20), cv::Point(-1, -1));
      cv::Mat Element = getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8), cv::Point(-1, -1));

      cv::erode(imagework, imagework, Element);
      cv::dilate(imagework, imagework, Element);

      cv::imshow(OPENCV_WINDOW3, imagework);

      //Element = getStructuringElement(cv::MORPH_RECT, cv::Size(500, 15), cv::Point(-1, -1));
      Element = getStructuringElement(cv::MORPH_RECT, cv::Size(180, 6), cv::Point(-1, -1));

      cv::erode(imagework, imagework, Element);
      cv::dilate(imagework, imagework, Element);

      //medianBlur(imagework, imagework, 11);
      medianBlur(imagework, imagework, 3);

      cv::imshow(OPENCV_WINDOW4, imagework);

      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours( imagework, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

      vector<RotatedRect> minRect( contours.size() );
      vector<Point2f>points;

      int cols = imagesrc.cols;
      int rows = imagesrc.rows;
      for( size_t i = 0; i< contours.size(); i++ ){
        minRect[i] = minAreaRect( Mat(contours[i]) );
        Point2f rect_points[4]; minRect[i].points( rect_points );

        for( int j = 0; j < 4; j++ ){
          if((rect_points[j].x < 5.0F || rect_points[j].x > cols-5.0F) && (rect_points[(j+1)%4].x < 5.0F || rect_points[(j+1)%4].x > cols-5.0F)){
            if(rect_points[j].y > 10.0F && rect_points[j].y < rows-10.0F && rect_points[(j+1)%4].y > 10.0F && rect_points[(j+1)%4].y < rows-10.0F){
              line( imagesrc, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 2, 8 );
              points.push_back(rect_points[j]);
            }
          }
        }
      }

      vector<Point2f>pointsR;
      vector<Point2f>pointsL;
      for( size_t i = 0; i< points.size(); i++ )
      {
        if(points[i].x < cols/2.0F) pointsL.push_back(points[i]);
        else pointsR.push_back(points[i]);
      }

      if(points.size()==8)
      {
        beltpoints.push_back(points[0]);
        beltpoints.push_back(points[1]);
        beltpoints.push_back(points[2]);
        beltpoints.push_back(points[3]);

        float max = 0;
        float min = rows;

        for( size_t i = 0; i< pointsR.size(); i++ )
        {
          if(pointsR[i].y > max) {
            max = pointsR[i].y;
            beltpoints.at(0) = pointsR[i];
          }

          if(pointsR[i].y < min) {
            min = pointsR[i].y;
            beltpoints.at(1) = pointsR[i];
          }
        }

        max = 0;
        min = rows;

        for( size_t i = 0; i< pointsL.size(); i++ )
        {
          if(pointsL[i].y > max) {
            max = pointsL[i].y;
            beltpoints.at(3) = pointsL[i];
          }

          if(pointsL[i].y < min) {
            min = pointsL[i].y;
            beltpoints.at(2) = pointsL[i];
          }
        }


        for ( size_t i = 0; i< points.size(); i++ ){
          circle(imagesrc, beltpoints[i],1, Scalar(255,0,0), 4, 8);
          for( size_t j = 0; j < 4; j++ )line( imagesrc, beltpoints[j], beltpoints[(j+1)%4], Scalar(0,255,0), 4, 8 );
        }
      }
      cv::imshow(OPENCV_WINDOW5, imagesrc);
      counter++;
    }
    cv::waitKey(3);
  }

  void step(){
    conveyorSystemRectPub.publish(conveyorSystemRect);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "conveyor_detector_node");
  ConveyorDetectorNode conveyorDetectorNode;
  ros::spin();
  /*ros::Rate loop_rate(10);

  while (ros::ok())
  {
    conveyorDetectorNode.step();

    ros::spinOnce();

    loop_rate.sleep();

  }*/
  return 0;
}
