cmake_minimum_required(VERSION 2.8.3)
project(betracker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  roscpp
  rospy
  std_msgs
  tf
all_msgs
darknet_ros_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_processing
  #CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
  CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS cv_bridge image_geometry image_transport roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
)


## Declare a C++ library
add_library(
  ${PROJECT_NAME}_lib
  src/${PROJECT_NAME}_lib/ukf.cpp
  src/${PROJECT_NAME}_lib/TFPoints.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)


add_executable(tracker src/tracking.cpp)
target_link_libraries(tracker
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
)
add_executable(tracker_ukf src/betracker_node.cpp)
target_link_libraries(tracker_ukf
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME}_lib
)


