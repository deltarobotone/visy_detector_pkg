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

/*static const std::string OPENCV_WINDOW1 = "Image window1";
static const std::string OPENCV_WINDOW2 = "Image window2";
static const std::string OPENCV_WINDOW3 = "Image window3";
static const std::string OPENCV_WINDOW4 = "Image window4";
static const std::string OPENCV_WINDOW5 = "Image window5";*/
using namespace cv;
using namespace std;

class ConveyorDetectorNode
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub_;
  cv::Mat imagework,imagesrc,imagegray,imagehsv;
  cv::Mat saturation, saturationf32,value, value32,chroma;
  std::vector<cv::Mat> imagesplit;
  vector<Point2d> conveyorSystemRect;
  uint counter = 0;
  uint searchLoops = 50;
  bool detected = false;
  bool done = false;
  Point2d centralLast;
  ros::ServiceServer detectConveyorSystemService;
  ros::Publisher conveyorSystemRectPub;
  visy_detector_pkg::ConveyorSystem conveyorSystemRectMsg;
  image_transport::Publisher imagePub;
  sensor_msgs::ImagePtr imageMsg;

public:
  ConveyorDetectorNode(): it(nh){
    detectConveyorSystemService = nh.advertiseService("detect_conveyor_system", &ConveyorDetectorNode::detectConveyorSystemCB,this);
    conveyorSystemRectPub = nh.advertise<visy_detector_pkg::ConveyorSystem>("conveyor_system_rect", 1);

  }
  ~ConveyorDetectorNode(){
    image_sub_.shutdown();
    imagePub.shutdown();
  }

  bool detectConveyorSystemCB(visy_detector_pkg::DetectConveyorSystem::Request  &req, visy_detector_pkg::DetectConveyorSystem::Response &res)
  {
    image_sub_ = it.subscribe("/raspicam_node/image", 1, &ConveyorDetectorNode::imageCb, this);
    imagePub = it.advertise("visy_image", 1);

    conveyorSystemRect.clear();
    conveyorSystemRect = vector<Point2d>{Point2d(),Point2d(),Point2d(),Point2d()};
    detected = false;
    done = false;
    conveyorSystemRectMsg.autodetected = detected;
    conveyorSystemRectMsg.done = done;
    counter = 0;
    return true;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try{cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);}
    catch (cv_bridge::Exception& e){return;}

    imagework = cv_ptr->image.clone();
    imagesrc = cv_ptr->image.clone();

    if(counter<searchLoops)
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

      cv::adaptiveThreshold(imagework,imagework,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,9,2);

      cv::Mat Element = getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4), cv::Point(-1, -1));

      cv::erode(imagework, imagework, Element);
      cv::dilate(imagework, imagework, Element);

      Element = getStructuringElement(cv::MORPH_RECT, cv::Size(140, 4), cv::Point(-1, -1));

      cv::erode(imagework, imagework, Element);
      cv::dilate(imagework, imagework, Element);

      medianBlur(imagework, imagework, 3);

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
              line( imagesrc, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 1, 8 );
              points.push_back(rect_points[j]);
            }
          }
        }
      }

      vector<Point2f>pointsR;
      vector<Point2f>pointsL;
      for( const auto & point:points){
        if(point.x < cols/2.0F) pointsL.push_back(point);
        else pointsR.push_back(point);
      }

      if(points.size()==8)
      {
        float meanL = 0.0F;
        for (const auto & point:pointsL) meanL += point.y;
        meanL = meanL/pointsL.size();

        float meanR = 0.0F;
        for (const auto & point:pointsR) meanR += point.y;
        meanR = meanR/pointsR.size();

        conveyorSystemRect.at(0).y = cols;
        conveyorSystemRect.at(1).y = 0.0;
        for (const auto & point:pointsL)
        {
          if(point.y > meanL && point.y < conveyorSystemRect.at(0).y)conveyorSystemRect.at(0)=point;
          if(point.y < meanL && point.y > conveyorSystemRect.at(1).y)conveyorSystemRect.at(1)=point;
        }

        conveyorSystemRect.at(3).y = cols;
        conveyorSystemRect.at(2).y = 0.0;
        for (const auto & point:pointsR)
        {
          if(point.y > meanR && point.y < conveyorSystemRect.at(3).y)conveyorSystemRect.at(3)=point;
          if(point.y < meanR && point.y > conveyorSystemRect.at(2).y)conveyorSystemRect.at(2)=point;
        }

        for ( size_t i = 0; i< points.size(); i++ ){
          circle(imagesrc, conveyorSystemRect[i],1, Scalar(255,0,0), 4, 8);
          for( size_t j = 0; j < 4; j++ )line( imagesrc, conveyorSystemRect[j], conveyorSystemRect[(j+1)%4], Scalar(0,255,0), 2, 8 );
        }

        detected = true;

      }
      counter++;
      imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imagesrc).toImageMsg();
      imagePub.publish(imageMsg);

    }
    else {
      if(detected == true){
        done=true;
        conveyorSystemRectMsg.autodetected = detected;
        conveyorSystemRectMsg.done = done;
      }
      else{
        conveyorSystemRect.at(0)=Point2f(0.0,200.0);
        conveyorSystemRect.at(1)=Point2f(0.0,110.0);
        conveyorSystemRect.at(2)=Point2f(410.0,110.0);
        conveyorSystemRect.at(3)=Point2f(410.0,200.0);
        done=true;
        conveyorSystemRectMsg.autodetected = detected;
        conveyorSystemRectMsg.done = done;
      }
      for(auto & points:conveyorSystemRectMsg.rect){
        points.x = conveyorSystemRect.front().x;
        points.y = conveyorSystemRect.front().y;
        conveyorSystemRect.erase(conveyorSystemRect.begin());
      }
      image_sub_.shutdown();
      imagePub.shutdown();
    }
    cv::waitKey(3);
  }
  void step(){
    if(done == true)conveyorSystemRectPub.publish(conveyorSystemRectMsg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "conveyor_detector_node");
  ConveyorDetectorNode conveyorDetectorNode;
  ros::Rate loop_rate(1);
  while (ros::ok()){
    ros::spinOnce();
    conveyorDetectorNode.step();
    loop_rate.sleep();
  }
  return 0;
}
