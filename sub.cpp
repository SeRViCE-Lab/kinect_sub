//  Subscriber to Thiemo Wiedemeyer's iai_kinect2 kinect meta-package
//  Code by olalekan ogunmolu
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
  std::cout << "          rosrun kinect_sub sub <name of stl plate> <sample_points> <leaf_size>" << std::endl;
  std::cout << "                                                                               " << std::endl;
  std::cout << "          <sample_points> is the number of points to use in sampling the visualizer" << std::endl;  
  std::cout << "          <leaf_size> is the size of the leaves to use for the visualizer" << std::endl;
  std::cout <<"                                " << std::endl;
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
    // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> syncPolicyExact; //not used

    imageMessageSub subImageColor, subImageDepth;
    camInfoSub subInfoCam, subInfoDepth;
    message_filters::Synchronizer<syncPolicy> sync;

    cv::Mat color, depth, ir, cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1;
    pcl::PCDWriter writer;

    std::mutex lock; 
    // std::mutex pcl_lock;  //storage for static lock
    ros::AsyncSpinner spinner;
    std::thread imageDispThread, modelDispThread;
    std::vector<int> opt;
    size_t frame;
    std::ostringstream oss;
    int argc;
    char** argv;
      
    std::vector<std::thread> threads;

    int thickness ;
    int rad ;
    const cv::Scalar line_color;
    const cv::Scalar circ_color;
    const cv::Scalar rect_color;

  public:
    Receiver(); //default constructor
    //copy constructor
    Receiver(int argc, char** argv)
      : updateCloud(false), updateImage(false), updateModel(false), save(false), basetopic("/kinect2"), 
      subNameColor(basetopic + "/qhd" + "/image_color_rect"), subNameDepth(basetopic + "/" + "qhd" + "/image_depth_rect"), topicCamInfoColor(basetopic + "/hd" + "/camera_info"), 
      subImageColor(nc, subNameColor, 1), subImageDepth(nc, subNameDepth, 1), subInfoCam(nc, topicCamInfoColor, 1),
      sync(syncPolicy(10), subImageColor, subImageDepth, subInfoCam), spinner(1), frame(0),  argc(argc), argv(argv), thickness(2), rad(8), line_color(cv::Scalar(0, 0, 255)),
      circ_color(cv::Scalar(255, 0, 0)), rect_color(cv::Scalar(0, 255, 0))
      {    
        ROS_INFO("Constructed Receiver");
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);        
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
      
        sync.registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3) );

        opt.push_back(cv::IMWRITE_JPEG_QUALITY);
        opt.push_back(100);
        opt.push_back(cv::IMWRITE_PNG_COMPRESSION);
        opt.push_back(1);
        opt.push_back(cv::IMWRITE_PNG_STRATEGY);
        opt.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
        opt.push_back(0);
        // loadSTL();
      }

    ~Receiver()
    {}   

    void run()
    {
      begin();
      end();
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
      ROS_INFO("started spinner");
      running = true;
      std::chrono::milliseconds duration(1);
      while(!updateImage || !updateCloud)
      {
        std::this_thread::sleep_for(duration);
      }
      cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
      cloud->height = color.rows;
      cloud->width = color.cols;
      cloud->is_dense = false;
      cloud->points.resize(cloud->height * cloud->width);
      makeLookup(this->color.cols, this->color.rows);
        
      //spawn the threads
      threads.push_back(std::thread(&Receiver::imageDisp, this));
      threads.push_back(std::thread(&Receiver::cloudViewer, this));

      //call join on each thread in turn
      std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join)); 
    }

    void end()
    {
      spinner.stop();         
      running = false;

      std::cout << "destroyed clouds visualizer" << std::endl;
    }

    void loadSTL()
    {
        int SAMPLE_POINTS_ = argc > 1 ? atoi(argv[2]) : 1000;
        float leaf_size    = argc > 2 ? atoi(argv[3]) : 100;
        pcl::PolygonMesh mesh;
        pcl::io::loadPolygonFileSTL (argv[1], mesh) ;
        std::vector<int> stl_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".stl");

        if (stl_file_indices.size () != 1)
        {
          ROS_WARN("Need a single output STL file to continue.\n");
        }

        vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();
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

      bool INTER_VIS = true;
      bool VIS = false;

      if (INTER_VIS)
       {
         pcl::visualization::PCLVisualizer vis;
         vis.addModelFromPolyData (polydata1, "mesh1", 0);
         vis.setRepresentationToSurfaceForAllActors ();
         ROS_INFO("Press 'q' to continue");
         vis.spin();
       }

       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
       uniform_sampling(polydata1, SAMPLE_POINTS_, *cloud_1);

       if(INTER_VIS)
       {
        pcl::visualization::PCLVisualizer vis_sampled;
          vis_sampled.addPointCloud(cloud_1);
          ROS_INFO("Press 'q' to continue");
          this->cloud_1 = cloud_1;
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
         cloud_init = !cloud_init;
        }         
          vis3.addPointCloud(res, "Model Voxel Cloud");
          vis3.resetCameraViewpoint("Model Voxel Cloud");
          vis3.registerKeyboardCallback(&Receiver::keyboardEvent, *this); 
          vis3.spinOnce();  
          ROS_INFO("Press 'q' to continue");
      }
    }

    void onMouse(int event, int x, int y, int flags, void* param)
    {
        char text[100];
        cv::Mat img3;

        if (event == CV_EVENT_LBUTTONDOWN)
        {
          cv::Vec3b p = this->color.at<cv::Vec3b>(y,x);
          sprintf(text, "R=%d, G=%d, B=%d", p[2], p[1], p[0]);
        }
        else if (event == CV_EVENT_RBUTTONDOWN)
        {
          cvtColor(this->color, img3, CV_BGR2HSV);
          cv::Vec3b p = img3.at<cv::Vec3b>(y,x);
          sprintf(text, "H=%d, S=%d, V=%d", p[0], p[1], p[2]);
        }
        else
          sprintf(text, "x=%d, y=%d", x, y);

        putText(this->color, text, cv::Point(5,15), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0));
    }


    void drawline(cv::Mat image, cv::Mat depth, cv::Point pt1, cv::Point pt2)
    {
      cv::line(image, pt1, pt2, line_color, thickness, cv::LINE_8, 0);
      cv::line(depth, pt1, pt2, line_color, thickness, cv::LINE_8, 0);
    }

    void drawcirc(cv::Mat image, cv::Mat depth, cv::Point circ)
    {      
      cv::circle(image, circ, rad, circ_color, thickness, cv::LINE_8, 0 );
      cv::circle(depth, circ, rad, circ_color, thickness, cv::LINE_8, 0 );
    }

    void drawrect(cv::Mat image, cv::Mat depth, cv::Point rect1, cv::Point rect2)
    {
      cv::rectangle(image, rect1, rect2, rect_color, thickness, cv::LINE_8, 0 ); 
      cv::rectangle(depth, rect1, rect2, rect_color, thickness, cv::LINE_8, 0 );   
    }


    void imageDisp()
    {
      cv::Mat color, depth;
      cv::Mat color_gray;   
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1;

      //hardcoded corners of 3d plate (found with gimp)
      cv::Point topleft(193, 168);
      cv::Point topright(662, 185);
      cv::Point botleft(186, 391);
      cv::Point botright(644, 438);

      //holes are listed left to right based on markings
      cv::Point circ11(265, 189);
      cv::Point circ12(423, 195);
      cv::Point circ13(595, 202);

      cv::Point rect211(220, 219);
      cv::Point rect212(236, 237);
      cv::Point rect221(373, 228);
      cv::Point rect222(389,246);
      cv::Point rect231(539, 235);
      cv::Point rect232(558, 253);

      cv::Point circ31(263, 269);
      cv::Point circ32(420, 280);
      cv::Point circ33(590, 290);

      cv::Point rect411(218, 298);
      cv::Point rect412(233, 315);
      cv::Point rect421(371, 309);
      cv::Point rect422(386, 327);
      cv::Point rect431(553, 336);
      cv::Point rect432(535, 319);

      cv::Point circ51(262, 349);
      cv::Point circ52(417, 362);
      cv::Point circ53(586, 378);


      cv::Point rect611(217, 375);
      cv::Point rect612(231, 390);
      cv::Point rect621(367, 390);
      cv::Point rect622(384, 408);
      cv::Point rect631(529, 408);
      cv::Point rect632(547, 423);

      uint16_t circ32_pt, rect42_pt, \
              botright_pt, topleft_pt;

      for(; running && ros::ok();)
      {
        if(updateImage)
        {
          lock.lock();
          color = this->color;
          depth = this->depth;
          updateImage = false;
          lock.unlock();
          cloud_1 = this->cloud_1;

          //draw the four lines around the plate
          drawline(color, depth, topleft, topright); drawline(color, depth, topleft, botleft); 
          drawline(color, depth, botleft, botright); drawline(color, depth, botright, topright);

          //draw circles around the objects
          drawcirc(color, depth, circ11); drawcirc(color, depth, circ12); drawcirc(color, depth, circ13);
          drawcirc(color, depth, circ31); drawcirc(color, depth, circ32); drawcirc(color, depth, circ33);
          drawcirc(color, depth, circ51); drawcirc(color, depth, circ52); drawcirc(color, depth, circ53);

          //draw the rects
          drawrect(color, depth, rect211, rect212); drawrect(color, depth, rect221, rect222); drawrect(color, depth, rect231, rect232); 
          drawrect(color, depth, rect411, rect412); drawrect(color, depth, rect421, rect422); drawrect(color, depth, rect431, rect432); 
          drawrect(color, depth, rect611, rect612); drawrect(color, depth, rect621, rect622); drawrect(color, depth, rect631, rect632); 

          //Pixel drift over time
          //10mm wide and 53mm deep circular and rectangular holes are in the middle of the depth map
          circ32_pt = depth.at<uint16_t>(circ32.y, circ32.x);  
          rect42_pt = depth.at<uint16_t>(floor(0.5*abs(rect422.y - rect421.y)), floor(0.5*abs(rect422.x - rect421.x)));  //this is the mid-point of the rectangles drawn
          //contour edges of the depth map
          topleft_pt = depth.at<uint16_t>(topleft.y, topleft.x);
          botright_pt = depth.at<uint16_t>(botright.y, botright.x);

          ROS_INFO_STREAM("middle of plate (" << circ32_pt << ", " << rect42_pt << ")");
          ROS_INFO_STREAM("edges of plate (" << topleft_pt << ", " << botright_pt << ")");

          // if(save)
          // {            
            cv::FileStorage fx;
            const cv::String& midplates = "midpoints.yaml";
            fx.open(midplates, cv::FileStorage::APPEND);
            fx << "circle32" << circ32_pt /*<< "\t" << "rect42" << rect42_pt*/;
            fx.release();

            cv::FileStorage fe;
            const cv::String& edges = "edges.yaml";
            fe.open(edges, cv::FileStorage::APPEND);
            fe << "topleft" << topleft_pt /*<< "\t" << "botright" << botright_pt*/;
            fe.release();
          // }

          cv::namedWindow("Hough Image", CV_WINDOW_AUTOSIZE);
          // cv::setMouseCallback("Hough Image", onMouse, void*);
          // cv::imshow("depth viewer", depth);
          cv::imshow("Hough Image", color);
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
            saveAll(cloud, color, depth, cloud_1);
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
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1;
            
      makeLookup(this->depth.cols, this->depth.rows);
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Cloud Viewer"));
      const std::string cloudName = "depth cloud";

      lock.lock();
      color = this->color;
      depth = this->depth;
      updateCloud = false;
      lock.unlock(); 
      cloud_1 = this->cloud_1;

      makeCloud(depth, color, cloud);

      void* viewer_void = NULL;
      viewer->setShowFPS(true);
      viewer->setPosition(0, 0);
      viewer->initCameraParameters();
      viewer->setSize(color.cols, color.rows);
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

          cloud_1 = this->cloud_1;

          makeCloud(depth, color, cloud);
          viewer->setSize(color.cols, color.rows);
          viewer->updatePointCloud(cloud, cloudName);

          if(save)
          {            
            save = false;
            saveAll(cloud, color, depth, cloud_1);
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

    void makeCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
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

    void saveAll(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_1)
    {
      oss.str("");
      oss << "./" << std::setfill('0') << std::setw(4) << frame;
      const std::string baseName = oss.str();
      const std::string cloudName = baseName + "_cloud.pcd";
      const std::string colorName = baseName + "_color.jpg";
      const std::string depthName = baseName + "_depth.png";
      // const std::string stl_cloud = basename + "_static.stl";

      ROS_INFO_STREAM("saving cloud: " << cloudName);
      writer.writeBinary(cloudName, *cloud);
      ROS_INFO_STREAM("saving color: " << colorName);
      cv::imwrite(colorName, color, opt);
      ROS_INFO_STREAM("saving depth: " << depthName);
      cv::imwrite(depthName, depth, opt);
      ROS_INFO_STREAM("saving cloud: " << "stl_cloud");
      writer.writeBinary("stl_cloud.pcd", *cloud_1);
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