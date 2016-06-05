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

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkVersion.h>
#include <vtkTriangle.h>
#include <vtkSTLReader.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>

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
  std::cout << "          rosrun kinect_sub sub <name of stl plate>" << std::endl;
  std::cout <<"                                " << std::endl;
}

namespace nodes
{
  enum Mode { color = 0,  depth = 1,  ir = 2,   mono = 3 };
}

class Receiver
{
  private:
    bool running, updateCloud, updateImage, updateModel, save;
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
    std::thread imageDispThread, modelDispThread;
    std::vector<int> opt;

    size_t frame;
    std::ostringstream oss;

    int argc;
    char** argv;

  public:
    Receiver(int argc, char** argv)
      : updateCloud(false), updateImage(false), updateModel(false), save(false), basetopic("/kinect2"), 
      subNameColor(basetopic + "/qhd" + "/image_color_rect"), subNameDepth(basetopic + "/" + "qhd" + "/image_depth_rect"), topicCamInfoColor(basetopic + "/hd" + "/camera_info"), 
      subImageColor(nc, subNameColor, 1), subImageDepth(nc, subNameDepth, 1), subInfoCam(nc, topicCamInfoColor, 1),
      sync(syncPolicy(10), subImageColor, subImageDepth, subInfoCam), spinner(1), frame(0),  argc(argc), argv(argv)
      {    
        ROS_INFO("Constructed Receiver");
        //initialize the K matrices or segfault
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);        
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
      
        sync.registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3) );

        // loadSTL(); 
        opt.push_back(cv::IMWRITE_JPEG_QUALITY);
        opt.push_back(100);
        opt.push_back(cv::IMWRITE_PNG_COMPRESSION);
        opt.push_back(1);
        opt.push_back(cv::IMWRITE_PNG_STRATEGY);
        opt.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
        opt.push_back(0);
      }

    ~Receiver()
    {}   

    void runSTL()
    {
      loadSTL();
    }

    void run()
    {
      ROS_INFO("Started Run");
      begin();
      end();
      ROS_INFO("called end()");
    }

  private:
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

    void begin()
    {
      spinner.start();
      // loadSTL();
      ROS_INFO("started spinner");
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
      // modelDispThread = std::thread(&Receiver::loadSTL, this); 
  
      cloudViewer();          
      ROS_INFO("after imageDispThread begin");   
      ros::waitForShutdown();      
      ROS_INFO("after waitForShutdown");
    }

    void end()
    {
      spinner.stop();
      ROS_INFO("Called end");
      running = false;
      imageDispThread.join();
      // modelDispThread.join();
      std::cout << "destroyed clouds visualizer" << std::endl;
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

    void loadSTL()
    {
        int SAMPLE_POINTS_ = 1000;
        float leaf_size = 100;
        pcl::PolygonMesh mesh;
        pcl::io::loadPolygonFileSTL (argv[1], mesh) ;
        std::vector<int> stl_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".stl");

        if (stl_file_indices.size () != 1)
        {
          ROS_INFO("Need a single output STL file to continue.\n");
          abort();
        }

        vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();;
        if (stl_file_indices.size () == 1)
        {
          pcl::PolygonMesh mesh;
          pcl::io::loadPolygonFileSTL (argv[stl_file_indices[0]], mesh);
          pcl::io::mesh2vtk (mesh, polydata1);
        }
        
        //make sure that the polygons are triangles!
        vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
        #if VTK_MAJOR_VERSION < 6
          triangleFilter->SetInput (polydata1);
        #else
          triangleFilter->SetInputData (polydata1);
        #endif
          triangleFilter->Update ();

      vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
      triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
      triangleMapper->Update();
      polydata1 = triangleMapper->GetInput();

      bool INTER_VIS = false;
      bool VIS = true;

      if (INTER_VIS)
       {
         pcl::visualization::PCLVisualizer vis;
         vis.addModelFromPolyData (polydata1, "mesh1", 0);
         vis.setRepresentationToSurfaceForAllActors ();
         vis.spin();
       }

       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
       uniform_sampling(polydata1, SAMPLE_POINTS_, *cloud_1);

       if(INTER_VIS)
       {
        pcl::visualization::PCLVisualizer vis_sampled;
        vis_sampled.addPointCloud(cloud_1);
        vis_sampled.spin();
       }

       //Voxelgrid
       pcl::VoxelGrid<pcl::PointXYZ> grid_;
       grid_.setInputCloud(cloud_1);
       grid_.setLeafSize(leaf_size, leaf_size, leaf_size);

       pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
       grid_.filter(*res);

      if(VIS)
      {         
        pcl::visualization::PCLVisualizer vis3; 
        bool cloud_init = false;

        if(!cloud_init)
        {        
         vis3.setShowFPS(true);
         vis3.setPosition(50, 50);
         vis3.setSize(depth.cols, depth.rows);
         // vis3.setBackgroundColor(0.2, 0.3, 0.3);
         cloud_init = !cloud_init;
        }         
          vis3.addPointCloud(res, "Model Voxel Cloud");
        vis3.resetCameraViewpoint("Model Voxel Cloud");
        vis3.registerKeyboardCallback(&Receiver::keyboardEvent, *this); 
          vis3.spin();  
      }
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
          // cv::imshow("depth viewer", depth);
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
          color = this->color;
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

    inline double uniform_deviate (int seed)
    {
      double ran = seed * (1.0 / (RAND_MAX + 1.0));
      return ran;
    }

    inline void randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p)
    {
      float r = static_cast<float> (Receiver::uniform_deviate (rand ()) * totalArea);

      std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
      vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());

      double A[3], B[3], C[3];
      vtkIdType npts = 0;
      vtkIdType *ptIds = NULL;
      polydata->GetCellPoints (el, npts, ptIds);
      polydata->GetPoint (ptIds[0], A);
      polydata->GetPoint (ptIds[1], B);
      polydata->GetPoint (ptIds[2], C);
      Receiver::randomPointTriangle (float (A[0]), float (A[1]), float (A[2]), 
                           float (B[0]), float (B[1]), float (B[2]), 
                           float (C[0]), float (C[1]), float (C[2]), p);
    }

    inline void randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, \
                         Eigen::Vector4f& p)
    {
      float r1 = static_cast<float> (uniform_deviate (rand ()));
      float r2 = static_cast<float> (uniform_deviate (rand ()));
      float r1sqr = sqrtf (r1);
      float OneMinR1Sqr = (1 - r1sqr);
      float OneMinR2 = (1 - r2);
      a1 *= OneMinR1Sqr;
      a2 *= OneMinR1Sqr;
      a3 *= OneMinR1Sqr;
      b1 *= OneMinR2;
      b2 *= OneMinR2;
      b3 *= OneMinR2;
      c1 = r1sqr * (r2 * c1 + b1) + a1;
      c2 = r1sqr * (r2 * c2 + b2) + a2;
      c3 = r1sqr * (r2 * c3 + b3) + a3;
      p[0] = c1;
      p[1] = c2;
      p[2] = c3;
      p[3] = 0;
    }

    void  uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ> & cloud_out)
    {
      polydata->BuildCells ();
      vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();

      double p1[3], p2[3], p3[3], totalArea = 0;
      std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
      size_t i = 0;
      vtkIdType npts = 0, *ptIds = NULL;
      for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
      {
        polydata->GetPoint (ptIds[0], p1);
        polydata->GetPoint (ptIds[1], p2);
        polydata->GetPoint (ptIds[2], p3);
        totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
        cumulativeAreas[i] = totalArea;
      }

      cloud_out.points.resize (n_samples);
      cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
      cloud_out.height = 1;

      for (i = 0; i < n_samples; i++)
      {
        Eigen::Vector4f p;
        randPSurface (polydata, &cumulativeAreas, totalArea, p);
        cloud_out.points[i].x = p[0];
        cloud_out.points[i].y = p[1];
        cloud_out.points[i].z = p[2];
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
      cv::imwrite(colorName, color, opt);
      ROS_INFO_STREAM("saving depth: " << depthName);
      cv::imwrite(depthName, depth, opt);
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

  ROS_INFO("==> point clouds rendering ...");

  Receiver receiver(argc, argv);
  receiver.run();

  if(!ros::ok())
  {
    return 0;
  }
  
  ros::shutdown();
}