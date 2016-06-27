cmake_minimum_required(VERSION 2.8.3)
project(kinect_sub)

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rostime
  sensor_msgs
  std_msgs
  tf
)

 find_package(Boost REQUIRED COMPONENTS system)
 find_package(PCL 1.2 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinect_sub
  CATKIN_DEPENDS cv_bridge message_runtime roscpp rostime sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}  
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sub src/sub.cpp)
add_executable(subAll src/subAll.cpp)
add_executable(modelLoader src/modelLoader.cpp)

target_link_libraries(sub   ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES}  ${kinect2_bridge_LIBRARIES})
target_link_libraries(modelLoader  ${catkin_LIBRARIES} ${PCL_LIBRARIES}  pcl_common pcl_io pcl_visualization pcl_filters ${VTK_LIBRARIES} )
target_link_libraries(subAll   ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES}  ${kinect2_bridge_LIBRARIES} )