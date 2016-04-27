#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("color view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_listener");
  ros::NodeHandle nc;
  cv::namedWindow("color view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nc);
  image_transport::Subscriber sub = it.subscribe("/kinect2/qhd/image_color", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("color view");
}