#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

const std::string encoding = std::string();

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("ir view", cv_bridge::toCvShare(msg, "16UC1")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ir_listener");
  ros::NodeHandle nr;
  cv::namedWindow("ir view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nr);
  image_transport::Subscriber sub = it.subscribe("/kinect2/sd/image_ir", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("ir view");
}