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

void help()
{
  std::cout << " USAGE: " << std::endl;
  std::cout << "          rosrun kinect_sub sub" << std::endl;
  std::cout <<"                                " << std::endl;
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
    std::string windowName;
    const std::string basetopic;
    std::string subNameColor, subNameDepth;
    cv_bridge::CvImagePtr cv_ptr;    
    const std::string topicCamInfoColor;

    typedef message_filters::Subscriber<sensor_msgs::Image> imageMessageSub;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> camInfoSub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> syncPolicy;
    // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> syncPolicy;

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
    Receiver()
      : updateCloud(false), updateImage(false), save(false), basetopic("/kinect2"), 
      subNameColor(basetopic + "/qhd" + "/image_color_rect"), subNameDepth(basetopic + "/" + "qhd" + "/image_depth_rect"), topicCamInfoColor(basetopic + "/hd" + "/camera_info"), 
      subImageColor(nc, subNameColor, 1), subImageDepth(nc, subNameDepth, 1), subInfoCam(nc, topicCamInfoColor, 1),
      sync(syncPolicy(10), subImageColor, subImageDepth, subInfoCam), spinner(3), frame(0)
      {    
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
      makeLookup(this->color.cols, this->color.rows);
      
      imageDispThread = std::thread(&Receiver::imageDisp, this);
      cloudViewer();
      
      ros::waitForShutdown();
    }

    void end()
    {
      spinner.stop();

      running = false;

      imageDispThread.join();
      std::cout << "destroyed clouds visualizer" << std::endl;
    }

    void callback(const sensor_msgs::ImageConstPtr imageColor, const sensor_msgs::ImageConstPtr imageDepth, const sensor_msgs::CameraInfoConstPtr colorInfo)
    {
      cv::Mat color, depth; 
  
      getCameraInfo(colorInfo, cameraMatrixColor);
      getImage(imageColor, color);
      getImage(imageDepth, depth);

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

    void getCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
    {
      double *itC = cameraMatrix.ptr<double>(0, 0);
      for(size_t i = 0; i < 9; ++i, ++itC)
      {
        *itC = cameraInfo->K[i];
      }
    }

    void getImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
    {
      cv_bridge::CvImageConstPtr pCvImage;
      pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
      pCvImage->image.copyTo(image);
    }

    void imageDisp()
    {
      cv::Mat color, depth;
      cv::Mat color_gray;   

      for(; running && ros::ok();)
      {
        if(updateImage)
        {
          lock.lock();
          color = this->color;
          depth = this->depth;
          updateImage = false;
          lock.unlock();

          //gray out and blur colored image
          cvtColor(color, color_gray, CV_BGR2GRAY);
          GaussianBlur(color_gray, color_gray, cv::Size(9, 9), 2, 2);
          std::vector<cv::Vec3f> circles;

          //reduce noise to avoid false circles
          HoughCircles(color_gray, circles, CV_HOUGH_GRADIENT, 1, color_gray.rows/8, 200, 100, 0, 0);

          //Draw the circles
          for (size_t i = 0; i < circles.size(); ++i)
          {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            //circle center
            circle(color, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
            circle(color, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
          }

          cv::namedWindow("Hough Image", CV_WINDOW_AUTOSIZE);

          cv::imshow("Hough Image", color);
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
            makeCloud(depth, color, cloud);
            saveAll(cloud, color, depth);
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
            
      makeLookup(this->depth.cols, this->depth.rows);
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Cloud Viewer"));
      const std::string cloudName = "depth cloud";

      lock.lock();
      color = this->color;
      depth = this->depth;
      updateCloud = false;
      lock.unlock(); 

      makeCloud(depth, color, cloud);

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

          makeCloud(depth, color, cloud);
          viewer->updatePointCloud(cloud, cloudName);

          if(save)
          {            
            save = false;
            saveAll(cloud, color, depth);
          }
        }          
        viewer->spinOnce(10);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100)); 
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

    void makeCloud(const cv::Mat &depth, cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) const
    {
      const float invalidPt = std::numeric_limits<float>::quiet_NaN();

      // #pragma omp parallel for
      for(int r = 0; r < depth.rows; ++r)
      {
        pcl::PointXYZRGBA *itPtr = &cloud->points[r * depth.cols];
        const uint16_t *itDepth = depth.ptr<uint16_t>(r);
        const cv::Vec3b *itColor = color.ptr<cv::Vec3b>(r);
        const float y = lookupY.at<float>(0, r);
        const float *itX = lookupX.ptr<float>();

        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itPtr, ++itColor, ++itDepth, ++itX)
        {
          register const float depthVal = *itDepth / 1000.0f;
          // Check for invalid measurements
          if(*itDepth == 0)
          { // not valid
            itPtr->x = itPtr->y = itPtr->z = invalidPt;
            itPtr->rgba = 0;
            continue;
          }
          itPtr->z = depthVal;
          itPtr->x = *itX * depthVal;
          itPtr->y = y * depthVal;
          itPtr->b = itColor->val[0];
          itPtr->g = itColor->val[1];
          itPtr->r = itColor->val[2];
          itPtr->a = 255;
        }
      }
    }

    void saveAll(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth)
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

    void makeLookup(size_t width, size_t height)
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

  help();

  ROS_INFO("Starting point clouds rendering ...");

  Receiver receiver;
  receiver.run();
  // receiver.imageDisp();

  if(!ros::ok())
  {
    return 0;
  }
  
  ros::shutdown();
}