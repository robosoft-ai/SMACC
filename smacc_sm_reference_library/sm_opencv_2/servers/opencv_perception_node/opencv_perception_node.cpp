#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>

ros::Publisher detectionPub;
ros::Publisher debugImagePub;
ros::Subscriber imageSub;

void segmentColor(const cv::Mat& inputRGB, int hueMean, int hueWindow, cv::Mat& out)
{
  cv::Mat hsvInput;
  cv::cvtColor(inputRGB, hsvInput, cv::COLOR_BGR2HSV);
  cv::inRange(hsvInput, cv::Scalar(hueMean - hueWindow, 20, 20), cv::Scalar(hueMean + hueWindow, 255, 255), out);
  cv::threshold(out, out, 100, 255, cv::THRESH_BINARY);
}

int testImage(cv::Mat& input, cv::Mat& debugImage, std::string colorName, int hueMean, int hueWindow)
{
  cv::Mat segmented;

  segmentColor(input, hueMean, hueWindow, segmented);

  cv::SimpleBlobDetector::Params params;
  // params.filterByArea= true;
  // params.minArea = 10;
  params.filterByArea = true;
  params.minArea = 100;
  params.maxArea = 100000;
  params.filterByColor = true;
  params.blobColor = 255;

  auto blobDetector = cv::SimpleBlobDetector::create(params);
  std::vector<cv::KeyPoint> blobs;
  blobDetector->detect(segmented, blobs);

  std::cout << "-------" << std::endl;
  for (auto& b : blobs)
  {
    std::cout << "blob " << colorName << " detected: " << b.size << std::endl;
  }

  if(!blobs.empty())
  { 
    for(auto& b: blobs)
    {
      cv::Rect r;
      float diameter = b.size;
      auto radius = diameter*0.5;
      r.x = b.pt.x - radius;
      r.y = b.pt.y - radius;
      r.width = diameter;
      r.height = diameter;
      cv::rectangle(debugImage,r, cv::Scalar(255,0,0),1 );
    }
  }

  // cv::imshow(colorName + " filter - "+ path, segmented);
  // cv::waitKey();

  return blobs.size();
}

int testRed(cv::Mat& input, cv::Mat& debugImage)
{
  return testImage(input, debugImage, "red", 130, 20);
}

int testBlue(cv::Mat& input, cv::Mat& debugImage)
{
  return testImage(input, debugImage, "blue", 10, 10);
}

int testGreen(cv::Mat& input, cv::Mat& debugImage)
{
  return testImage(input, debugImage, "green", 50, 10);
}

int testRed(std::string path, cv::Mat& debugImage)
{
  cv::Mat input = cv::imread(path);
  return testImage(input, debugImage, "red", 130, 20);
}

int testBlue(std::string path, cv::Mat& debugImage)
{
  cv::Mat input = cv::imread(path);
  return testImage(input, debugImage, "blue", 10, 10);
}

int testGreen(std::string path, cv::Mat& debugImage)
{
  cv::Mat input = cv::imread(path);
  return testImage(input, debugImage, "green", 50, 10);
}

void update()
{
}

void callback(const sensor_msgs::Image& img)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(img, "rgb8");
  
  cv::Mat image = cv_image->image;
  cv_bridge::CvImage debugImageBridge;
  debugImageBridge.encoding = "rgb8";
  debugImageBridge.image = image.clone();

  int detectedColor = 0;
  if (testRed(image, debugImageBridge.image) > 0)
  {
    detectedColor = 1;
  }
  
  if (testGreen(image, debugImageBridge.image) > 0)
  {
    detectedColor = 2;
  }
  
  if (testBlue(image, debugImageBridge.image) > 0)
  {
    detectedColor = 3;
  }

  std_msgs::Int32 detectedColorMsg;
  detectedColorMsg.data = detectedColor;
  detectionPub.publish(detectedColorMsg);

  debugImageBridge.header = img.header;
  debugImagePub.publish(debugImageBridge.toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "blob_detector_node");
  ros::NodeHandle nh;

  detectionPub = nh.advertise<std_msgs::Int32>("detected_color", 1);
  imageSub = nh.subscribe("/image_raw", 1, callback);
  debugImagePub = nh.advertise<sensor_msgs::Image>("/opencv_debug_image", 1);

  ros::Rate r(10);

  while (ros::ok())
  {
    update();
    ros::spinOnce();
    r.sleep();
  }

  /*
  testRed("../../red1.png");
  testRed("../../green1.png");
  testRed("../../blue1.png");
  testRed("../../red2.png");

  testGreen("../../red1.png");
  testGreen("../../green1.png");
  testGreen("../../blue1.png");
  testGreen("../../red2.png");

  testBlue("../../red1.png");
  testBlue("../../green1.png");
  testBlue("../../blue1.png");
  testBlue("../../red2.png");
  */
}