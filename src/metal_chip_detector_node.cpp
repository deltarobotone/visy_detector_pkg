#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "visy_detector_pkg/StartMetalChipDetector.h"
#include "visy_detector_pkg/StopMetalChipDetector.h"
#include "visy_detector_pkg/SelectImage.h"
#include "visy_detector_pkg/MetalChip.h"
#include "visy_detector_pkg/ConveyorSystem.h"

using namespace cv;
using namespace std;

class MetalChipDetectorNode
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub_;
  cv::Mat imagework,imagesrc,imagegray,imagehsv;
  cv::Mat saturation, saturationf32,value, value32,chroma;
  std::vector<cv::Mat> imagesplit;
  vector<Point2f> conveyorSystemRect;
  ros::ServiceServer startMetalChipDetectorService,stopMetalChipDetectorService,selectImageService;
  ros::Subscriber conveyorSystemRectSub;
  ros::Publisher metalChipPub;
  Point2d centralLast;
  sensor_msgs::Image imageLast;
  image_transport::Publisher imagePub;
  sensor_msgs::ImagePtr imageMsg;

  enum Image{SOURCE,HSV,CHROMA,ADAPTHRESH,ERODE,DILATE,MEDIAN,DETECTED};
  ulong selectedImage = SOURCE;

public:
  MetalChipDetectorNode(): it(nh){
    conveyorSystemRectSub = nh.subscribe("conveyor_system_rect", 1, &MetalChipDetectorNode::conveyorSystemRectCb,this);
    metalChipPub = nh.advertise<visy_detector_pkg::MetalChip>("metal_chip", 1);
    startMetalChipDetectorService = nh.advertiseService("start_metalchip_detector", &MetalChipDetectorNode::startMetalChipDetectorCB,this);
    stopMetalChipDetectorService = nh.advertiseService("stop_metalchip_detector", &MetalChipDetectorNode::stopMetalChipDetectorCB,this);
    selectImageService = nh.advertiseService("select_image", &MetalChipDetectorNode::selectImageCB,this);
    imagePub = it.advertise("visy_image", 1);
  }
  ~MetalChipDetectorNode(){
    imagePub.shutdown();
  }

  bool startMetalChipDetectorCB(visy_detector_pkg::StartMetalChipDetector::Request  &req, visy_detector_pkg::StartMetalChipDetector::Response &res){
    image_sub_ = it.subscribe("/raspicam_node/image", 1, &MetalChipDetectorNode::imageCb, this);
    return true;
  }
  bool stopMetalChipDetectorCB(visy_detector_pkg::StopMetalChipDetector::Request  &req, visy_detector_pkg::StopMetalChipDetector::Response &res){
    image_sub_.shutdown();
    return true;
  }
  bool selectImageCB(visy_detector_pkg::SelectImage::Request  &req, visy_detector_pkg::SelectImage::Response &res){
    if(selectedImage < sizeof(Image) && req.direction == req.NEXT)selectedImage++;
    if(selectedImage > 0 && req.direction == req.BACK)selectedImage--;
    return true;
  }

  int getHueValue(Mat &hsv){
    int hbins = 30, sbins = 32;
    int histSize[] = { hbins, sbins }; // Quantize the hue to 30 levels and the saturation to 32 levels
    float hranges[] = { 0, 180 }; // hue varies from 0 to 179, see cvtColor
    float sranges[] = { 0, 256 }; // saturation varies from 0 (black-gray-white) to 255 (pure spectrum color)
    const float* ranges[] = { hranges, sranges };
    MatND hist;
    int channels[] = { 0, 1 }; // compute the histogram from the 0-th and 1-st channels

    calcHist(&hsv, 1, channels, Mat(), hist, 2, histSize, ranges,true,false);
    double maxVal = 0;
    minMaxLoc(hist, 0, &maxVal, 0, 0);

    int scale = 10;
    Mat histImg = Mat::zeros(sbins*scale, hbins*scale, CV_8UC3);
    int intensity, maxinstensity = 0;
    int hue = 0;

    for (int h = 0; h < hbins; h++){
      for (int s = 0; s < sbins; s++){
        float binVal = hist.at<float>(h, s);
        intensity = cvRound(binVal * 255 / maxVal);
        rectangle(histImg, Point(h*scale, s*scale),Point((h + 1)*scale - 1, (s + 1)*scale - 1),Scalar::all(intensity),CV_FILLED);
        if (intensity > maxinstensity){
          maxinstensity = intensity;
          hue = h*(180/hbins);
        }
      }
    }
    return hue;
  }

  void conveyorSystemRectCb(const visy_detector_pkg::ConveyorSystemConstPtr& conveyorSystem){
    for(auto const& point:conveyorSystem->rect){
      Point2d p;
      p.x = point.x;
      p.y = point.y;
      conveyorSystemRect.push_back(p);
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image){

    cv_bridge::CvImagePtr cv_ptr;

    try{cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);}
    catch (cv_bridge::Exception& e){return;}

    imagework = cv_ptr->image.clone();
    imagesrc = cv_ptr->image.clone();

    if(selectedImage==Image::SOURCE)publishImage(imagework);

    cv::Rect roi(conveyorSystemRect[0],conveyorSystemRect[2]);

    imagework = imagesrc(roi);
    imagesrc = imagework.clone();

    cv::cvtColor(imagework, imagehsv, CV_BGR2HSV);
    if(selectedImage==Image::HSV)publishImage(imagehsv);

    split(imagehsv, imagesplit);

    saturation = imagesplit[1].clone();
    saturation.convertTo(saturationf32, CV_32FC1, 1. / 255);

    value = imagesplit[2].clone();
    value.convertTo(value32, CV_32FC1, 1. / 255);

    add(value32, saturationf32, chroma);

    chroma.convertTo(chroma, CV_8UC1, 200);

    imagework = chroma.clone();

    if(selectedImage==Image::CHROMA)publishImage(imagework);

    cv::adaptiveThreshold(imagework,imagework,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,5,2);

    if(selectedImage==Image::ADAPTHRESH)publishImage(imagework);

    cv::Mat Element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8), cv::Point(-1, -1));

    cv::erode(imagework, imagework, Element);
    if(selectedImage==Image::DILATE)publishImage(imagework);

    cv::dilate(imagework, imagework, Element);
    if(selectedImage==Image::ERODE)publishImage(imagework);

    medianBlur(imagework, imagework, 3);
    if(selectedImage==Image::MEDIAN)publishImage(imagework);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( imagework, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    int i = 0;
    for( auto const& contour:contours){

      double area = contourArea(contour);
      double contourLength = arcLength(contour, true);
      double formfactorCircle = ((4 * 3.141 * area) / pow(contourLength,2.0));

      if (formfactorCircle > 0.80 && formfactorCircle < 0.95 && area > 2900.0){
        Moments M = moments(contour);
        Point2d central = Point2d(M.m10 / M.m00, M.m01 / M.m00);
        RotatedRect rects = minAreaRect(contour);

        getRectSubPix(imagehsv, rects.size, rects.center, imagework);

        visy_detector_pkg::MetalChip metalchipMsg;
        metalchipMsg.hue = getHueValue(imagework);
        metalchipMsg.pos[0] = uint(central.x);
        metalchipMsg.pos[1] = uint(central.y);
        metalchipMsg.imageTime = image->header.stamp;

        metalchipMsg.header.stamp = ros::Time::now();
        metalChipPub.publish(metalchipMsg);

        drawContours(imagesrc,contours,i,Scalar(0, 255, 0), 2);
        circle(imagesrc, central, 4, Scalar(0, 255, 0), 2);
      }
      i++;
    }
    if(selectedImage==Image::DETECTED)publishImage(imagesrc);
    cv::waitKey(1);

  }

  void publishImage(cv::Mat &img){
    cvtColor(img, img, COLOR_GRAY2RGB);
    imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    imagePub.publish(imageMsg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "metal_chip_detector_node");
  MetalChipDetectorNode metalChipDetectorNode;
  ros::spin();
  return 0;
}
