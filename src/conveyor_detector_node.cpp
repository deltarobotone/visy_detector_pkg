#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "visy_detector_pkg/DetectConveyorSystem.h"
#include "visy_detector_pkg/ConveyorSystem.h"

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
  cv::Mat imagework,imagesrc,imagegray,imagehsv;
  cv::Mat saturation, saturationf32,value, value32,chroma;
  std::vector<cv::Mat> imagesplit;
  vector<Point2d> conveyorSystemRect;
  uint counter = 0;
  Point2d centralLast;
  ros::ServiceServer detectConveyorSystemService;
  ros::Publisher conveyorSystemRectPub;

public:
  ConveyorDetectorNode(): it_(nh_){
    detectConveyorSystemService = nh_.advertiseService("detect_conveyor_system", &ConveyorDetectorNode::detectConveyorSystemCB,this);
    conveyorSystemRectPub = nh_.advertise<visy_detector_pkg::ConveyorSystem>("conveyor_system_rect", 1);
  }
  ~ConveyorDetectorNode(){
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
    while(counter<10){
      ros::spinOnce();
    }
    image_sub_.shutdown();

    visy_detector_pkg::ConveyorSystem conveyorSystemRectMsg;

    for(auto & points:conveyorSystemRectMsg.rect){
      points.x = conveyorSystemRect.at(0).x;
    }

    conveyorSystemRectPub.publish(conveyorSystemRectMsg);

    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
    cv::destroyWindow(OPENCV_WINDOW3);
    cv::destroyWindow(OPENCV_WINDOW4);
    cv::destroyWindow(OPENCV_WINDOW5);

    return true;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try{cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);}
    catch (cv_bridge::Exception& e){return;}

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
      cv::Mat Element = getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4), cv::Point(-1, -1));

      cv::erode(imagework, imagework, Element);
      cv::dilate(imagework, imagework, Element);

      cv::imshow(OPENCV_WINDOW3, imagework);

      //Element = getStructuringElement(cv::MORPH_RECT, cv::Size(500, 15), cv::Point(-1, -1));
      Element = getStructuringElement(cv::MORPH_RECT, cv::Size(140, 4), cv::Point(-1, -1));

      cv::erode(imagework, imagework, Element);
      cv::dilate(imagework, imagework, Element);

      //medianBlur(imagework, imagework, 11);
      medianBlur(imagework, imagework, 3);

      cv::imshow(OPENCV_WINDOW4, imagework);

      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(imagework, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

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
      for( size_t i = 0; i< points.size(); i++ ){
        if(points[i].x < cols/2.0F) pointsL.push_back(points[i]);
        else pointsR.push_back(points[i]);
      }

      if(points.size()==8)
      {
        conveyorSystemRect.push_back(points[0]);
        conveyorSystemRect.push_back(points[1]);
        conveyorSystemRect.push_back(points[2]);
        conveyorSystemRect.push_back(points[3]);

        float max = 0;
        float min = rows;

        for( size_t i = 0; i< pointsR.size(); i++ )
        {
          if(pointsR[i].y > max) {
            max = pointsR[i].y;
            conveyorSystemRect.at(0) = pointsR[i];
          }

          if(pointsR[i].y < min) {
            min = pointsR[i].y;
            conveyorSystemRect.at(1) = pointsR[i];
          }
        }

        max = 0;
        min = rows;

        for( size_t i = 0; i< pointsL.size(); i++ )
        {
          if(pointsL[i].y > max) {
            max = pointsL[i].y;
            conveyorSystemRect.at(3) = pointsL[i];
          }

          if(pointsL[i].y < min) {
            min = pointsL[i].y;
            conveyorSystemRect.at(2) = pointsL[i];
          }
        }

        for ( size_t i = 0; i< points.size(); i++ ){
          circle(imagesrc, conveyorSystemRect[i],1, Scalar(255,0,0), 4, 8);
          for( size_t j = 0; j < 4; j++ )line( imagesrc, conveyorSystemRect[j], conveyorSystemRect[(j+1)%4], Scalar(0,255,0), 4, 8 );
        }
      }
      cv::imshow(OPENCV_WINDOW5, imagesrc);
      counter++;
    }
    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "conveyor_detector_node");
  ConveyorDetectorNode conveyorDetectorNode;
  ros::spin();
  return 0;
}
