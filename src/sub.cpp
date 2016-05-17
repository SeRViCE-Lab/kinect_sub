#include <ros/ros.h>
#include <iostream>
#include <cassert>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

std::string imageEncoding;
std::string windowName;

std::string nodeName;
std::string topicName;
std::string imageFormat;

cv::Mat temp;
cv_bridge::CvImagePtr cv_ptr, depth_ptr, ir_ptr;

void help()
{
  std::cout << " USAGE: " << std::endl;
  std::cout << "          rosrun kinect_sub <topicName> <imageFormat>" << std::endl;
  std::cout <<"           topicName = {<hd> | <qhd> | <sd> }         " << std::endl;
  std::cout <<"           imageFormat = {<ir> | <depth_rect>  | <color> } " << std::endl;
  std::cout <<"           imageFormat = {<ir_rect> | <depth_rect>  | <color_rect> } " << std::endl;
}

namespace nodes
{
  enum Mode { color = 0,  depth = 1,  ir = 2,   mono = 3 };
}

class Receiver
{
  ros::NodeHandle nc;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;

  public:
    Receiver(std::string topicName, std::string imageFormat)
      : running(false), it(nc), topicName_(topicName), imageFormat_(imageFormat)
    {         
      const std::string basetopic = "/kinect2";
      std::string subName    = basetopic + "/" + topicName_ + "/" + "image_" + imageFormat_  ;
      sub = it.subscribe(subName, 1, &Receiver::imageCallback, this);
      ROS_INFO_STREAM("Subscribing to: " << subName);
      windowName = imageFormat_ + " viewer";
      cv::namedWindow(windowName);      
      cv::startWindowThread();
    }

    ~Receiver()
    {      
      cv::destroyWindow(windowName);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      try
      {
        ROS_INFO("msg frame_id: %s", msg->header.frame_id.c_str());
        ROS_INFO("msg step %u",  msg->step);
        ROS_INFO("msg encoding %s", msg->encoding.c_str());

        if(imageFormat == "color_rect") //color
          {  
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::circle(cv_ptr->image, cv::Point(30, 30), 10, CV_RGB(255,0,0));  //draw a circle on the circle image
          }
        else if(msg->encoding.c_str() == "16UC1") //ir or depth
          {  
            if(msg->header.frame_id.c_str() == "kinect2_rgb_optical_frame") //depth
            {          
              cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            }
            else if(msg->header.frame_id.c_str() == "kinect2_ir_optical_frame") //ir)
            {          
              cv_ptr= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            }
          }

        cv::imshow(windowName, cv_bridge::toCvShare(msg, msg->encoding.c_str())->image);
        cv::waitKey(3);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }
    }

  private:
    bool running;
    std::string topicName_, imageFormat_;
};


int main(int argc, char **argv)
{  
  nodes::Mode modeVar; 

  switch(modeVar)
  {
    case nodes::color :
      nodeName = "colorGrabber"; break;
    case nodes::depth :
      nodeName = "depthGrabber"; break;
    case nodes::ir :
      nodeName = "irGrabber"; break;
    case nodes::mono :
      nodeName = "monoGrabber"; break;
    default :
      ROS_INFO("Ouch! This ain't yo' lane. Take the next exit. Wrong nodename supplied' ");
  }
  ros::init(argc, argv, nodeName, ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  if (argc <3 )
    {
      ROS_INFO("You must supply the topic arguments"); 
      help();
      return 0;
    }

  for(size_t i = 1; i < (size_t)argc; ++i)  
  {      
      topicName = argv[1];
      imageFormat = argv[2]; 
  }

  assert(imageFormat == "ir" || imageFormat == "ir_rect" || imageFormat == "color" || imageFormat == "color_rect" || imageFormat == "depth" || \
                         imageFormat == "depth_rect" || imageFormat == "mono" ||imageFormat ==   "mono_rect");
  Receiver receiver(topicName, imageFormat);

  ros::spin();
  return 0;
}