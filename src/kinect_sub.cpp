#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread/mutex.hpp> 

#include <string>

// using namespace sensor_msgs;
// using namespace message_filters;

bool updateImage(false);
bool useCompressed = false;
bool useExact(true);
std::ostringstream oss;
const std::string& encoding = std::string();
const std::string& base_topic = "/kinect2/qhd/image_color";
boost::mutex lock;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo> ExactSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

cv::Mat readImage(const sensor_msgs::ImageConstPtr& colorImage, cv::Mat& image) 
{
  cv_bridge::CvImageConstPtr pCvImage;
  pCvImage = cv_bridge::toCvShare(colorImage, encoding);
  pCvImage->image.copyTo(image);
  ROS_INFO_STREAM("pCvImage" << pCvImage);
  return image;
}

void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr camInfo, cv::Mat &cameraMatrix)
{
  double *itC = cameraMatrix.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    *itC = camInfo->K[i];
  }
}

void callback(const sensor_msgs::ImageConstPtr& colorImage, const sensor_msgs::CameraInfo::ConstPtr camInfo)
{
  cv::Mat color, cameraMatrixColor;
  color = readImage(colorImage, color);
  readCameraInfo(camInfo, cameraMatrixColor);
  try
  {
    // IR image input
    if(color.type() == CV_16U)
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      // cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }    

/*    lock.lock();
    this->color = color;
    updateImage = true;
    lock.unlock();*/

    oss << "starting...";
    cv::namedWindow("color subscriber");

    for(; ros::ok();)
    {


      // cv::imshow("color subscriber", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::imshow("color subscriber", color);
      //cv::waitKey(30);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", colorImage->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("color_subscriber", cv::WINDOW_NORMAL);
  cv::startWindowThread();

  uint32_t queueSize;
  image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");


  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter *subImageColor;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  subImageColor = new image_transport::SubscriberFilter(it, base_topic, queueSize, hints);
  subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, "camInfo", queueSize);

/*  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, base_topic, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "camInfo", 1);
  TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync(image_sub, info_sub, 10);
  //TimeSynchronizer<Image, CameraInfo> sync(subImageColor, subCameraInfoColor, queueSize);
  sync.registerCallback(boost::bind(&callback, _1, _2));*/
  if(useExact)
  {
    syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subCameraInfoColor);
    syncExact->registerCallback(boost::bind(&callback, _1, _2));
  }
  else
  {
    syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subCameraInfoColor);
    syncApproximate->registerCallback(boost::bind(&callback, _1, _2));
  }

  // image_transport::Subscriber sub = it.subscribe("/kinect2/hd/image", 1, callback);
  ros::spin();
  cv::destroyWindow("color_subscriber");
}