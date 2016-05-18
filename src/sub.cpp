#include <ros/ros.h>
#include <iostream>
#include <cassert>
#include <limits>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

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

void help()
{
  std::cout << " USAGE: " << std::endl;
  std::cout << "          rosrun kinect_sub <topicName> <imageFormat>" << std::endl;
  std::cout <<"           topicName = {<hd> | <qhd> | <sd> }         " << std::endl;
  std::cout <<"           imageFormat = {<ir> | <depth_rect>  | <color> } " << std::endl;
  std::cout <<"           imageFormat = {<ir_rect> | <depth_rect>  | <color_rect> } " << std::endl;
}


class Receiver
{
  ros::NodeHandle nc;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  // image_transport::SubscriberFilter subColor;
  ros::Subscriber subColor;

  public:
    Receiver(std::string topicName, std::string imageFormat)
      : running(false), it(nc), topicName_(topicName), imageFormat_(imageFormat), GoDepth(false),
          GoColor(false), GoIr(false), screen_height(640), screen_width(480), viewport(0), 
          queueSize(5), basetopic("/kinect2")
    {         
      const std::string basetopic = "/kinect2";
      std::string subName    = basetopic + "/" + topicName_ + "/" + "image_" + imageFormat_  ;
      sub = it.subscribe(subName, 1, &Receiver::imageCallback, this);
      this->subCams();
      ROS_INFO_STREAM("Subscribing to: " << subName);
      windowName = imageFormat_ + " viewer";
      cv::namedWindow(windowName);   
      viewer = this->createCloud(cloud_ptr, msg);   
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
          }
      }

      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }
    }

    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& colormsg)
    {
      cv::Mat color, depth;      
      ROS_INFO_STREAM("colorInfo msg: " << colormsg);
    }

    void subCams()
    {      
      const std::string subCamInfoColor = basetopic + "/hd" + "/camera_info";
      const std::string subCamInfoDepth = basetopic + "/sd" + "/camera_info";
      const sensor_msgs::CameraInfo::ConstPtr colormsg;
      const ros::TransportHints& hints = ros::TransportHints().maxDatagramSize(1000) ;
      uint32_t second = 1;
/*      boost::function<void(const sensor_msgs::CameraInfo::ConstPtr)> camInfoFunction;
      //bind the methods
      camInfoFunction = boost::bind(&Receiver::camInfoCallback, _1);*/     //void (*)(const ImageConstPtr&)
      subColor = it.subscribe(subCamInfoColor, second, this->camInfoCallback(colormsg), hints);
      ROS_INFO("I subscribed to color cam info");
      // ros::Subscriber sub = nc.subscribe(subCamInfoColor, 1, &Receiver::camInfoCallback, &foo_object);
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> createCloud (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, sensor_msgs::ImageConstPtr msg)
    {
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Depth Viewer"));
      const std::string cloudName = "depth cloud";

      cloudDisp(depth, color, msg, cloud_ptr);

      viewer->initCameraParameters ();
      viewer->setBackgroundColor (0.2, 0.3, 0.3);
      viewer->setSize(screen_height, screen_width);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName, viewport);
      viewer->addCoordinateSystem (0.5);    //don't want no cylinder
      viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, cloudName);
      viewer->setSize(640, 480);
      return (viewer);
    }

    void cloudDisp(cv::Mat depth, cv::Mat color, sensor_msgs::ImageConstPtr msg, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr)
    {  
      const float badPoint = std::numeric_limits<float>::quiet_NaN();

      #pragma omp parallel for 
      for (int i = 0; i < depth.rows; ++i)
      {
        pcl::PointXYZRGBA *itP = &cloud_ptr->points[i * depth.cols];
        const uint16_t *itD = depth.ptr<uint16_t>(i);        
        const cv::Vec3b *itC = color.ptr<cv::Vec3b>(i);
        const float y = lookupY.at<float>(0, i);
        const float *itX = lookupX.ptr<float>();

        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD)
        {
          const float depthVal = *itD/1000.0f;

          //we check for invalid floats within depth map
          if(*itD == 0)
          {
            //inValids
            itP->x = itP->y = itP->z = badPoint;
            itP->rgba = 0;
            continue;
          }
          itP->z = depthVal;
          itP->x = *itX * depthVal;
          itP->y = y * depthVal;          
          itP->b = itC->val[0];
          itP->g = itC->val[1];
          itP->r = itC->val[2];
          itP->a = 255;
          cloud_ptr.push_back(*itP);
        }        
        cloud_ptr->height = depth.rows;
        cloud_ptr->width = depth.cols;
        cloud_ptr->is_dense = false;
      }

      if (GoDepth)
      {
        viewer = createCloud(cloud_ptr, msg);        
        //process point_clouds
        ROS_INFO_STREAM("Rows: " << cloud_ptr->height << " Heght: " << cloud_ptr->width);
      }
    }

/*  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
       *it = (r - cy) * fy;
    }
        
    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }*/

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
    std::string topicName_, imageFormat_;
    cv::Mat color, depth, ir;
    bool GoDepth, GoColor, GoIr; bool running;
    const int screen_height, screen_width, viewport;
    cv_bridge::CvImagePtr cv_ptr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;
    sensor_msgs::ImageConstPtr msg; 
    const size_t queueSize;    
    const std::string basetopic;
    std::string windowName;
};


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
  // pcl::PointCloud<pcl::PointXYZ>::ConstPtr depth_cloud_ptr;
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  // viewer = receiver.createCloud();
  ros::spin();
  return 0;
}