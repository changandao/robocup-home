cmake_minimum_required(VERSION 2.8.3)
project(yolo_box)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  all_msgs
  darknet_ros_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(yolo_box src/yolo_box.cpp)
target_link_libraries(yolo_box ${catkin_LIBRARIES})