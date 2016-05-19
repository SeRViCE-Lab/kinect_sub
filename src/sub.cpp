//  Subscriber to Thiemo Wiedemeyer's iai_kinect2 kinect meta-package
//  Code by olalekan ogunmolu
//  Amazon Robotics LLC
//  May 2016, MA, USA
#include <ros/ros.h>
#include <iostream>
#include <cassert>
#include <mutex>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

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
  private:
    bool running, updateCloud, GoColor, GoDepth, GoIr;
    ros::NodeHandle nc;
    // image_transport::ImageTransport it;
    std::string topicName, imageFormat, windowName;
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
    cv::Mat lookupX, lookupY;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::mutex lock;

  public:
    Receiver(std::string topicName, std::string imageFormat)
      : running(false), updateCloud(false), GoColor(false), GoDepth(false), GoIr(false), topicName(topicName), 
      imageFormat(imageFormat), windowName(imageFormat + " viewer"), basetopic("/kinect2"), 
      subName(basetopic + "/" + topicName + "/" + "image_" + imageFormat ), topicCamInfoColor(basetopic + "/hd" + "/camera_info"), 
      subImage(nc, subName, 1), subCam(nc, topicCamInfoColor, 1),
      sync(syncPolicy(10), subImage, subCam)
      {
        sync.registerCallback(boost::bind(&Receiver::callback, this, _1, _2) );
        cv::namedWindow(windowName);      
        cv::startWindowThread();
        //initialize the K matrices or segfault
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);

/*        lock.lock();
        this->color = color;
        this->depth = depth;
        // updateImage = true;
        updateCloud = true;
        lock.unlock();*/
      }

    ~Receiver()
    {            
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
      // ROS_INFO("msg encoding %s", msg->encoding.c_str());
      readCameraInfo(colorInfo, cameraMatrixColor);
      ROS_INFO_STREAM("Camera Matrix Color : \n" << cameraMatrixColor);        
      running = true;    

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
          cloudDisp();
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

    void cloudDisp()
    {      
      cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      cloud->height = depth.rows;
      cloud->width = depth.cols;
      cloud->is_dense = false;
      cloud->points.resize(cloud->height * cloud->width);          
      createLookup(this->depth.cols, this->depth.rows);
      cloudViewer();
    }

    void cloudViewer()
    {
      cv::Mat depth;
      pcl::visualization::PCLVisualizer::Ptr witch(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
      const std::string cloudName = "depth cloud";

      lock.lock();
      color = this->color;
      depth = this->depth;
      updateCloud = false;
      lock.unlock();

      createCloud(depth, cloud);

      witch->setShowFPS(true);
      witch->setPosition(0, 0);
      witch->initCameraParameters();
      witch->setSize(depth.cols, depth.rows);
      witch->setBackgroundColor(0.2, 0.3, 0.3);
      witch->setCameraPosition(0, 0, 0, 0, -1, 0);
      witch->registerKeyboardCallback(&Receiver::keyboardEvent, *this);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> depth_color_handler(cloud, 255, 0, 255);
      witch->addPointCloud(cloud, depth_color_handler, cloudName);
      witch->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);      

      for(; running && ros::ok();)
      {
        if(updateCloud)
        {
          lock.lock();
          color = this->color;
          depth = this->depth;          
          // updateCloud = false;
          lock.unlock();

          ROS_INFO("updateCloud %d, ", updateCloud);

          // witch->removePointCloud(cloudName);          
          // createCloud(depth, cloud);
          witch->updatePointCloud(cloud, cloudName);
        }        
        updateCloud = true;
        witch->spinOnce(10);
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
      }
      witch->close();
    }

    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
    {
      if(event.keyUp())
      {
        switch(event.getKeyCode())
        {
        case 27:
        case 'q':
          running = false;
          break;
        case ' ':
        // case 's':
        //   save = true;
         break;
        }
      }
    }

    void createCloud(const cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
    {
      const float badPoint = std::numeric_limits<float>::quiet_NaN();

      // #pragma omp parallel for
      for(int r = 0; r < depth.rows; ++r)
      {
        pcl::PointXYZ *itP = &cloud->points[r * depth.cols];
        const uint16_t *itD = depth.ptr<uint16_t>(r);
        // const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
        const float y = lookupY.at<float>(0, r);
        const float *itX = lookupX.ptr<float>();

        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itX)
        {
          register const float depthValue = *itD / 1000.0f;
          // Check for invalid measurements
          if(*itD == 0)
          {
            // not valid
            itP->x = itP->y = itP->z = badPoint;
            continue;
          }
          itP->z = depthValue;
          itP->x = *itX * depthValue;
          itP->y = y * depthValue;
        }
      }
    }

    void createLookup(size_t width, size_t height)
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
    }
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
      ROS_INFO(" Default ");
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

  assert(imageFormat == "ir" or "ir_rect" or "color" or "color_rect" or "depth" or "depth_rect" or "depth_rect/compressed" or "depth/compressed" or  "mono" or  "mono_rect"\
          and "\nimage format should be of one of the above" );

  Receiver receiver(topicName, imageFormat);

  while(ros::ok())
  {
    ros::spin();    
  }
  return 0;
}