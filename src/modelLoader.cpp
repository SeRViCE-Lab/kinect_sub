#include <ros/ros.h>
#include <vtkVersion.h>
#include <vtkTriangle.h>
#include <vtkSTLReader.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>



inline double uniform_deviate (int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
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

inline void randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p)
{
  float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);

  std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
  vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType *ptIds = NULL;
  polydata->GetCellPoints (el, npts, ptIds);
  polydata->GetPoint (ptIds[0], A);
  polydata->GetPoint (ptIds[1], B);
  polydata->GetPoint (ptIds[2], C);
  randomPointTriangle (float (A[0]), float (A[1]), float (A[2]), 
                       float (B[0]), float (B[1]), float (B[2]), 
                       float (C[0]), float (C[1]), float (C[2]), p);
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

void loadSTL(int argc, char* argv[])
{
    int SAMPLE_POINTS_ = /*argc > 1 ? atoi(argv[2]) : */1000;
    float leaf_size    = /*argc > 2 ? atoi(argv[3]) :*/ 100;
    pcl::PolygonMesh mesh;    
    ROS_INFO_STREAM("argv "<< argv[1] << ", " << argv[2] << ", " << argv[3]);
    pcl::io::loadPolygonFileSTL (argv[1], mesh) ;
    ROS_INFO("loaded polygon");
    std::vector<int> stl_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".STL");
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
     vis3.setSize(640, 480);
     cloud_init = !cloud_init;
    }         
      vis3.addPointCloud(res, "Model Voxel Cloud");
      vis3.resetCameraViewpoint("Model Voxel Cloud");
      vis3.spinOnce();  
      ROS_INFO("Press 'q' to continue");
  }
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "model_loader");
	ROS_INFO("initialized ros");
	loadSTL(argc, argv);
ROS_INFO("loaded stl");
	ros::spinOnce();

	return 0;
}