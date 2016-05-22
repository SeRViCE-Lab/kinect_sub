//  Subscriber to Thiemo Wiedemeyer's iai_kinect2 kinect meta-package
//  Code by olalekan ogunmolu
//  Amazon Robotics LLC
//  May 2016, MA, USA
#include <ros/ros.h>
#include <iostream>
#include <cassert>
#include <mutex>
#include <thread>

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
    bool running, updateCloud, updateImage, save;
    ros::NodeHandle nc;
    // image_transport::ImageTransport it;
    std::string topicName, imageFormat, windowName;
    const std::string basetopic;
    std::string subNameColor, subNameDepth;
    cv_bridge::CvImagePtr cv_ptr;    
    const std::string topicCamInfoColor;

    typedef message_filters::Subscriber<sensor_msgs::Image> imageMessageSub;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> camInfoSub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> syncPolicy;

    imageMessageSub subImageColor, subImageDepth;
    camInfoSub subInfoCam, subInfoDepth;
    message_filters::Synchronizer<syncPolicy> sync;

    cv::Mat color, depth, ir, cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    pcl::PCDWriter writer;
    std::mutex lock; 
    ros::AsyncSpinner spinner;
    std::thread imageDispThread;
    std::vector<int> params;

    size_t frame;
    std::ostringstream oss;

  public:
    Receiver(std::string topicName, std::string imageFormat)
      : updateCloud(false), updateImage(false), save(false), topicName(topicName), 
      imageFormat(imageFormat), basetopic("/kinect2"), subNameColor(basetopic + "/qhd" + "/image_color_rect"), 
      subNameDepth(basetopic + "/" + "qhd" + "/image_depth_rect"), topicCamInfoColor(basetopic + "/hd" + "/camera_info"), 
      subImageColor(nc, subNameColor, 1), subImageDepth(nc, subNameDepth, 1), subInfoCam(nc, topicCamInfoColor, 1),
      sync(syncPolicy(10), subImageColor, subImageDepth, subInfoCam), spinner(0)
      {    
        cv::startWindowThread();
        //initialize the K matrices or segfault
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);        
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
      
        sync.registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3) );

        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(100);
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(1);
        params.push_back(cv::IMWRITE_PNG_STRATEGY);
        params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
        params.push_back(0);
      }

    ~Receiver()
    {            
      cv::destroyWindow(windowName);
    }

    void run()
    {
      begin();
      end();
    }

  private:
    void begin()
    {
      spinner.start();
      running = true;
      std::chrono::milliseconds duration(1);
      while(!updateImage || !updateCloud)
      {
        if(!ros::ok())
        {
          return;
        }
        std::this_thread::sleep_for(duration);
      }
      cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
      cloud->height = color.rows;
      cloud->width = color.cols;
      cloud->is_dense = false;
      cloud->points.resize(cloud->height * cloud->width);
      createLookup(this->color.cols, this->color.rows);

      imageDispThread = std::thread(&Receiver::imageDisp, this);
      cloudViewer();
    }

    void end()
    {
      spinner.stop();

      running = false;
      imageDispThread.join();
    }

    void callback(const sensor_msgs::ImageConstPtr imageColor, const sensor_msgs::ImageConstPtr imageDepth, const sensor_msgs::CameraInfoConstPtr colorInfo)
    {
      cv::Mat color, depth; 
  
      readCameraInfo(colorInfo, cameraMatrixColor);
      readImage(imageColor, color);
      readImage(imageDepth, depth);

      // IR image input
      if(color.type() == CV_16U)
      {
        cv::Mat tmp;
        color.convertTo(tmp, CV_8U, 0.02);
        cv::cvtColor(tmp, color, CV_GRAY2BGR);
      }

      lock.lock();
      this->color = color;
      this->depth = depth;
      updateImage = true;
      updateCloud = true;
      lock.unlock();
    }

    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
    {
      double *itC = cameraMatrix.ptr<double>(0, 0);
      for(size_t i = 0; i < 9; ++i, ++itC)
      {
        *itC = cameraInfo->K[i];
      }
    }

    void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
    {
      cv_bridge::CvImageConstPtr pCvImage;
      pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
      pCvImage->image.copyTo(image);
    }

    void imageDisp()
    {
      cv::Mat color, depth;   

      for(; running && ros::ok();)
      {
        if(updateImage)
        {
          lock.lock();
          color = this->color;
          depth = this->depth;
          updateImage = false;
          lock.unlock();

          cv::imshow("color viewer", color);
          cv::imshow("depth viewer", depth);
        }

        int key = cv::waitKey(1);
        switch(key & 0xFF)
        {
          case 27:
          case 'q':
            running = false;
            break;
          case ' ':
          case 's':
            createCloud(depth, color, cloud);
            saveCloudAndImages(cloud, color, depth);
            save = true;
            break;
        }
      }
      cv::destroyAllWindows();
      cv::waitKey(100);
    }

    void cloudViewer()
    {     
      cv::Mat color, depth;
            
      createLookup(this->depth.cols, this->depth.rows);
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Cloud Viewer"));
      const std::string cloudName = "depth cloud";

      lock.lock();
      color = this->color;
      depth = this->depth;
      updateCloud = false;
      lock.unlock(); 

      createCloud(depth, color, cloud);

      void* viewer_void = NULL;
      viewer->setShowFPS(true);
      viewer->setPosition(0, 0);
      viewer->initCameraParameters();
      viewer->setSize(depth.cols, depth.rows);
      viewer->setBackgroundColor(0.2, 0.3, 0.3);
      viewer->setCameraPosition(0, 0, 0, 0, -1, 0);       
      viewer->registerKeyboardCallback(&Receiver::keyboardEvent, *this); 
      viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, cloudName);
      
      for(; running && ros::ok() ;)
      {                      
        if(updateCloud)
        {
          lock.lock();
          depth = this->depth;        
          updateCloud = false;
          lock.unlock();

          createCloud(depth, color, cloud);
          viewer->updatePointCloud(cloud, cloudName);

          if(save)
          {            
            save = false;
            saveCloudAndImages(cloud, color, depth);
          }
        }          
        viewer->spinOnce(10);
        boost::this_thread::sleep(boost::posix_time::microseconds(100)); 
      } 
      viewer->close();
    }

    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void * viewer_void)
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
        case 's':
          save = true;
         break;
        }
      }
    }

    void createCloud(const cv::Mat &depth, cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) const
    {
      const float badPoint = std::numeric_limits<float>::quiet_NaN();

      // #pragma omp parallel for
      for(int r = 0; r < depth.rows; ++r)
      {
        pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
        const uint16_t *itD = depth.ptr<uint16_t>(r);
        const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
        const float y = lookupY.at<float>(0, r);
        const float *itX = lookupX.ptr<float>();

        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itC, ++itD, ++itX)
        {
          register const float depthValue = *itD / 1000.0f;
          // Check for invalid measurements
          if(*itD == 0)
          { // not valid
            itP->x = itP->y = itP->z = badPoint;
            itP->rgba = 0;
            continue;
          }
          itP->z = depthValue;
          itP->x = *itX * depthValue;
          itP->y = y * depthValue;
          itP->b = itC->val[0];
          itP->g = itC->val[1];
          itP->r = itC->val[2];
          itP->a = 255;
        }
      }
    }

    void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth)
    {
      oss.str("");
      oss << "./" << std::setfill('0') << std::setw(4) << frame;
      const std::string baseName = oss.str();
      const std::string cloudName = baseName + "_cloud.pcd";
      const std::string colorName = baseName + "_color.jpg";
      const std::string depthName = baseName + "_depth.png";

      ROS_INFO_STREAM("saving cloud: " << cloudName);
      writer.writeBinary(cloudName, *cloud);
      ROS_INFO_STREAM("saving color: " << colorName);
      cv::imwrite(colorName, color, params);
      ROS_INFO_STREAM("saving depth: " << depthName);
      cv::imwrite(depthName, depth, params);
      ROS_INFO_STREAM("saving complete!");
      ++frame;
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
  ros::init(argc, argv, "subscriber", ros::init_options::AnonymousName);

  bool running;
  if(ros::ok())  { running = true; }
  else if(!ros::ok) { return 0;}

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

  ROS_INFO("Starting point clouds rendering ...");

  receiver.run();
  ros::shutdown();
  ros::spin();

  return 0;
}