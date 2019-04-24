cmake_minimum_required(VERSION 2.8)
project(obsavoid)
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
std_srvs
  geometry_msgs
  sensor_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
  image_transport
  gazebo_ros
)

catkin_package( 
CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
  pcl_ros
  pcl_conversions
  image_transport
  gazebo_ros
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(p3atObsAvoid SHARED src/p3atObsAvoid.cpp)
add_library(assistMath   SHARED src/assistMath.cpp)



#pcl
find_package(PCL REQUIRED COMPONENT common io)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#eigen
include_directories("/usr/include/eigen3/")

#cv
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})
#target_link_libraries( ${PROJECT_NAME}  )
#add_executable(imageBasics imageBasics.cpp)
#target_link_libraries(imageBasics ${OpenCV_LIBS})

#add_executable(pcl_sample src/pcl_sample.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkint_LIBRARIES} )

#gazebo
find_package(gazebo REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})


add_executable(${PROJECT_NAME}
  src/main.cpp
  src/Octree.hpp
  src/p3atObsAvoid.h
  src/p3atObsAvoid.cpp
  src/assistMath.cpp
  src/assistMath.h
  src/Pioneer3AT.cpp
#  pointcloud_viewer.cpp
  )
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  p3atObsAvoid
  assistMath 
)

add_executable(pointcloud_viewer_node
pointcloud_viewer.cpp)

target_link_libraries(pointcloud_viewer_node ${catkin_LIBRARIES})














