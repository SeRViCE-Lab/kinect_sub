//  Subscriber to Thiemo Wiedemeyer's iai_kinect2 kinect meta-package
//  Code by olalekan ogunmolu
//  Amazon Robotics LLC
//  May 2016, MA, USA
#include <ros/ros.h>
#include <iostream>
#include <cassert>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

std::string nodeName;
std::string topicName;
std::string imageFormat;


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
  public:
    Receiver(std::string topicName, std::string imageFormat)
      : running(false), GoColor(false), GoDepth(false), GoIr(false), topicName_(topicName), 
      imageFormat_(imageFormat), windowName(imageFormat_ + " viewer"), basetopic("/kinect2"), 
      subName(basetopic + "/" + topicName_ + "/" + "image_" + imageFormat_ ), topicCamInfoColor(basetopic + "/hd" + "/camera_info"), 
      subImage(nc, subName, 1), subCam(nc, topicCamInfoColor, 1),
      sync(syncPolicy(10), subImage, subCam)
      {
        sync.registerCallback(boost::bind(&Receiver::callback, this, _1, _2) );
        cv::namedWindow(windowName);      
        cv::startWindowThread();
        running = true;
        //initialize the K matrices or segfault
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
      }

    ~Receiver()
    {            
      running = false;
      cv::destroyWindow(windowName);
    }
    
    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
    {
      double *itC = cameraMatrix.ptr<double>(0, 0);
      for(size_t i = 0; i < 9; ++i, ++itC)
      {
        *itC = cameraInfo->K[i];
      }
    }

    void callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr colorInfo)
    {
      ROS_INFO("msg frame_id: %s", msg->header.frame_id.c_str());
      // ROS_INFO("msg step %u",  msg->step);
      ROS_INFO("msg encoding %s", msg->encoding.c_str());
      readCameraInfo(colorInfo, cameraMatrixColor);
      ROS_INFO_STREAM("Camera Matrix Color : \n" << cameraMatrixColor);      

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
        }
      else if(msg->header.frame_id.c_str() == "kinect2_ir_optical_frame") //ir)
        { 
          GoIr = true;       
          cv_ptr= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
          ir = cv_ptr->image;
          imageDisp();
        }
      else
        {
          ROS_INFO("Incorrect image format supplied");
        }
    }

    void imageDisp()
    {
      if(GoColor) 
      {         
        cv::imshow(windowName, color);
        cv::waitKey(3);
      }
      else if(GoDepth)
      {          
        cv::imshow(windowName, depth);        
        cv::waitKey(3);
      }
      else if(GoIr)
      {          
        cv::imshow(windowName, color);
        cv::waitKey(3);
      }
      else{ ROS_INFO("No valid Mat supplied to display"); }
    }

  private:
    bool running, GoColor, GoDepth, GoIr;
    ros::NodeHandle nc;
    // image_transport::ImageTransport it;
    std::string topicName_, imageFormat_, windowName;
    const std::string basetopic;
    std::string subName;
    cv_bridge::CvImagePtr cv_ptr;    
    const std::string topicCamInfoColor;

    typedef message_filters::Subscriber<sensor_msgs::Image> imageMessageSub;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> camInfoSub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> syncPolicy;

    imageMessageSub subImage;
    camInfoSub subCam;
    message_filters::Synchronizer<syncPolicy> sync;

    cv::Mat color, depth, ir, cameraMatrixColor;
    float *colorMatrix;
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

  if(imageFormat == "ir_rect" || imageFormat == "ir" && topicName != "sd")
    {
      ROS_INFO("for ir Images, topic name must be \"sd\"");
      abort();
    }

  assert(imageFormat == "ir" || imageFormat == "ir_rect" || imageFormat == "color" || imageFormat == "color_rect" || imageFormat == "depth" || \
                         imageFormat == "depth_rect" || imageFormat == "mono" ||imageFormat ==   "mono_rect");

  Receiver receiver(topicName, imageFormat);

  while(ros::ok())
  {
    ros::spin();    
  }
  return 0;
}