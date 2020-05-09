#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <visy_neopixel_pkg/StatusBar.h>

/*static const std::string OPENCV_WINDOW1 = "Image window1";
static const std::string OPENCV_WINDOW2 = "Image window2";
static const std::string OPENCV_WINDOW3 = "Image window3";
static const std::string OPENCV_WINDOW4 = "Image window4";
static const std::string OPENCV_WINDOW5 = "Image window5";*/
using namespace cv;
using namespace std;

class ImageProcessingNode
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
  ros::ServiceClient statusBarClient;
  visy_neopixel_pkg::StatusBar statusBar;
  Point2f MC, MC_PRE;

public:
  ImageProcessingNode(): it_(nh_)
  {
    image_sub_ = it_.subscribe("/raspicam_node/image", 1, &ImageProcessingNode::imageCb, this);
    statusBarClient = nh_.serviceClient<visy_neopixel_pkg::StatusBar>("ctrl_status_bar");
    /*cv::namedWindow(OPENCV_WINDOW1);
    cv::namedWindow(OPENCV_WINDOW2);
    cv::namedWindow(OPENCV_WINDOW3);
    cv::namedWindow(OPENCV_WINDOW4);
    cv::namedWindow(OPENCV_WINDOW5);*/
  }
  ~ImageProcessingNode()
  {
    /*cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
    cv::destroyWindow(OPENCV_WINDOW3);
    cv::destroyWindow(OPENCV_WINDOW4);
    cv::destroyWindow(OPENCV_WINDOW5);*/
  }

  void findColours(Mat &hsv, int &colour)
  {
    // Quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 30, sbins = 32;
    int histSize[] = { hbins, sbins };
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    MatND hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = { 0, 1 };

    calcHist(&hsv, 1, channels, Mat(), // do not use mask
             hist, 2, histSize, ranges,
             true, // the histogram is uniform
             false);
    double maxVal = 0;
    minMaxLoc(hist, 0, &maxVal, 0, 0);

    int scale = 10;
    Mat histImg = Mat::zeros(sbins*scale, hbins * 10, CV_8UC3);
    int maxinstensity = 0;
    int hue = 0;

    for (int h = 0; h < hbins; h++){
      for (int s = 0; s < sbins; s++)
      {
        float binVal = hist.at<float>(h, s);
        int intensity = cvRound(binVal * 255 / maxVal);

        rectangle(histImg, Point(h*scale, s*scale),
                  Point((h + 1)*scale - 1, (s + 1)*scale - 1),
                  Scalar::all(intensity),
                  CV_FILLED);

        if (intensity > maxinstensity)
        {
          maxinstensity = intensity;
          hue = h*6;
        }

      }
    }
    //colour=1 rot
    if ((hue >= 0 && hue < 10) || (hue <= 180 && hue > 160)) colour = 1;
    //colour=2 gelb
    if (hue >= 15 && hue < 45) colour = 2;
    //colour=3 blau
    if (hue >= 100 && hue < 130) colour = 3;
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

    if(check == 0)
    {
      statusBar.request.ctrl = statusBar.request.FLOW_ONE_FREE_CW;
      statusBar.request.r = 0;
      statusBar.request.g = 0;
      statusBar.request.b = 0;
      statusBar.request.w = 255;

      statusBarClient.call(statusBar);
    }

    if(check > 5 && check < 20)
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

      //--cv::imshow(OPENCV_WINDOW1, imagework);

      cv::adaptiveThreshold(imagework,imagework,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,21,2);

      //--cv::imshow(OPENCV_WINDOW2, imagework);

      cv::Mat Element = getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20), cv::Point(-1, -1));

      cv::erode(imagework, imagework, Element);
      cv::dilate(imagework, imagework, Element);

      Element = getStructuringElement(cv::MORPH_RECT, cv::Size(500, 15), cv::Point(-1, -1));

      cv::erode(imagework, imagework, Element);
      cv::dilate(imagework, imagework, Element);

      medianBlur(imagework, imagework, 11);

      //--cv::imshow(OPENCV_WINDOW3, imagework);

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
              line( imagesrc, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 4, 8 );
              points.push_back(rect_points[j]);
            }
          }
        }
      }

      if(points.size()==8)
      {
        beltpoints.push_back(points[2]);
        beltpoints.push_back(points[3]);
        beltpoints.push_back(points[4]);
        beltpoints.push_back(points[5]);
      }

      for ( size_t i = 0; i< beltpoints.size(); i++ ){
        circle(imagesrc, beltpoints[i],1, Scalar(255,0,0), 4, 8);
        for( size_t j = 0; j < 4; j++ )line( imagesrc, beltpoints[j], beltpoints[(j+1)%4], Scalar(0,255,0), 4, 8 );
      }
      //--cv::imshow(OPENCV_WINDOW4, imagesrc);
      if(check == 10)
      {
        //cv::destroyWindow(OPENCV_WINDOW1);
        //cv::destroyWindow(OPENCV_WINDOW2);
        //cv::destroyWindow(OPENCV_WINDOW3);
        //cv::destroyWindow(OPENCV_WINDOW4);
      }
    }
    else if(check > 20) {

      start = std::chrono::system_clock::now();

      cv::Rect roi(beltpoints[0],beltpoints[2]);

      imagework = imagesrc(roi);
      imagesrc = imagework.clone();

      cv::cvtColor(imagework, imagehsv, CV_BGR2HSV);

      split(imagehsv, imagesplit);

      saturation = imagesplit[1].clone();
      saturation.convertTo(saturationf32, CV_32FC1, 1. / 255);

      value = imagesplit[2].clone();
      value.convertTo(value32, CV_32FC1, 1. / 255);

      add(value32, saturationf32, chroma);

      chroma.convertTo(chroma, CV_8UC1, 200);

      imagework = chroma.clone();

      //--cv::imshow(OPENCV_WINDOW2, imagework);

      cv::adaptiveThreshold(imagework,imagework,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,7,2);

      //--cv::imshow(OPENCV_WINDOW3, imagework);

      cv::Mat Element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20), cv::Point(-1, -1));

      cv::erode(imagework, imagework, Element);
      cv::dilate(imagework, imagework, Element);

      medianBlur(imagework, imagework, 11);

      //--cv::imshow(OPENCV_WINDOW4, imagework);

      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours( imagework, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

      for( size_t i = 0; i< contours.size(); i++ ){
        Scalar color = Scalar(255,255,255);
        drawContours( imagesrc, contours, i, color, 2, 8, hierarchy, 0, Point() );
      }

      double a, l, formf;

      for( size_t i = 0; i< contours.size(); i++ ){
        a = contourArea(contours[i]);
        l = arcLength(contours[i], true);

        formf = ((4 * 3.141 * a) / (l*l));

        //ROS_ERROR("%f",formf);

        if (formf > 0.84 && formf < 0.92 && a > 25000.0)
        {
          Moments M;
          M = moments(contours[i]);

          Point2f MC = Point2f(M.m10 / M.m00, M.m01 / M.m00);

          int colour = 0;
          RotatedRect rects = minAreaRect(contours[i]);

          getRectSubPix(imagehsv, rects.size, rects.center, imagework);
          findColours(imagework, colour);

          counter++;

          double x_diff = abs(MC.x - MC_PRE.x);

          MC_PRE = MC;

          int latenz_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(start-end).count();

          //cout << "latenz: " << latenz_seconds << endl;

          //cout << "posdiff: " << x_diff << endl;

          double v = x_diff / latenz_seconds;

          //cout << "velocity: " << v << endl;

          end = std::chrono::system_clock::now();

          statusBar.request.ctrl = statusBar.request.FULL;

          if (colour == 1){
            statusBar.request.r = 255;
            statusBar.request.g = 0;
            statusBar.request.b = 0;
            statusBar.request.w = 0;
          }
          else if (colour == 2){
            statusBar.request.r = 255;
            statusBar.request.g = 255;
            statusBar.request.b = 0;
            statusBar.request.w = 0;

          }
          else if (colour == 3){
            statusBar.request.r = 0;
            statusBar.request.g = 0;
            statusBar.request.b = 255;
            statusBar.request.w = 0;
          }

          statusBarClient.call(statusBar);

          circle(imagesrc, MC, 4, Scalar(255, 0, 0), 4, 8, 0);
        }


      }
      if(contours.size()==0 && statusBar.request.ctrl != statusBar.request.FLOW_DOUBLE_TOP)
      {
        statusBar.request.ctrl = statusBar.request.FLOW_DOUBLE_TOP;
        statusBar.request.r = 0;
        statusBar.request.g = 255;
        statusBar.request.b = 0;
        statusBar.request.w = 0;

        statusBarClient.call(statusBar);
      }
      //--cv::imshow(OPENCV_WINDOW1, imagesrc);

    }
    check++;
    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_processing_node");
  ImageProcessingNode imageProcessingNode;
  ros::spin();
  return 0;
}
