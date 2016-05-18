#include <ros/ros.h>
#include <iostream>
#include <cassert>
#include <limits>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


std::string nodeName;
std::string topicName;
std::string imageFormat;

using namespace message_filters;

void help()
{
  std::cout << " USAGE: " << std::endl;
  std::cout << "          rosrun kinect_sub <topicName> <imageFormat>" << std::endl;
  std::cout <<"           topicName = {<hd> | <qhd> | <sd> }         " << std::endl;
  std::cout <<"           imageFormat = {<ir> | <depth_rect>  | <color> } " << std::endl;
  std::cout <<"           imageFormat = {<ir_rect> | <depth_rect>  | <color_rect> } " << std::endl;
}



    void callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& colorInfo)
    {
      try
      {
        ROS_INFO("msg frame_id: %s", msg->header.frame_id.c_str());
        ROS_INFO("msg step %u",  msg->step);
        ROS_INFO("msg encoding %s", msg->encoding.c_str());

        ROS_INFO_STREAM("colorInfo" << *colorInfo);
/*
        if(imageFormat == "color_rect") //color
          {  
            GoColor = true;
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            color = cv_ptr->image;        
            imageDisp();
          }
        else if(imageFormat == "depth_rect" || "depth") //depth
          {    
            GoDepth = true;     
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            depth = cv_ptr->image;
            imageDisp();
            cloudDisp(depth, color, msg, cloud_ptr);
          }
        else if(msg->header.frame_id.c_str() == "kinect2_ir_optical_frame") //ir)
          { 
            GoIr = true;       
            cv_ptr= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            ir = cv_ptr->image;
            imageDisp();
            cloudDisp(ir, color, msg, cloud_ptr);
          }
        else
          {
            ROS_INFO("Incorrect image format supplied");
          }*/
      }

      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }
    }



namespace nodes
{
  enum Mode { color = 0,  depth = 1,  ir = 2,   mono = 3 };
}

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

  ros::NodeHandle nc;

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

  if(imageFormat == "ir_rect" || imageFormat == "ir" && topicName != "sd")
    {
      ROS_INFO("for ir Images, topic name must be \"sd\"");
      abort();
    }

  assert(imageFormat == "ir" || imageFormat == "ir_rect" || imageFormat == "color" || imageFormat == "color_rect" || imageFormat == "depth" || \
                         imageFormat == "depth_rect" || imageFormat == "mono" ||imageFormat ==   "mono_rect");


  const std::string basetopic = "/kinect2";
  std::string subName    = basetopic + "/" + topicName + "/" + "image_" + imageFormat ;
  const std::string topicCamInfoColor = basetopic + "/hd" + "/camera_info";

  message_filters::Subscriber<sensor_msgs::Image> imageSub(nc, subName, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> colorInfoSub(nc, topicCamInfoColor, 1);
  TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync(imageSub, colorInfoSub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2)); 

  ros::spin();
  return 0;
}