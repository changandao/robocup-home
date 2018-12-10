cmake_minimum_required(VERSION 2.8.3)
project(perception)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)


find_package(catkin REQUIRED COMPONENTS 
  roscpp
  darknet_ros_msgs
  sensor_msgs
  cv_bridge
  image_transport
  all_msgs
  geometry_msgs
  control_msgs
  actionlib
  tf
  tum_alle_common_msgs
  message_filters
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs all_msgs geometry_msgs control_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(yolo_roi src/yolo_roi.cpp)
target_link_libraries(yolo_roi ${catkin_LIBRARIES})

add_executable(depth2pointcloud src/depth2pointcloud.cpp)
target_link_libraries(depth2pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(head_tracking src/head_tracking.cpp)
target_link_libraries(head_tracking ${catkin_LIBRARIES})

add_executable(online_collecting src/online_collecting.cpp)
target_link_libraries(online_collecting ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(online_learning src/online_learning.cpp)
target_link_libraries(online_learning ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(prediction src/predict.cpp)
#target_link_libraries(prediction ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(object_sending src/object_sending.cpp)
#target_link_libraries(object_sending ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(predictshow src/predictshow.cpp)
target_link_libraries(predictshow ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(object_detection src/object_detection.cpp)
target_link_libraries(object_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(depthprocessing src/DepthProcessing.cpp)
target_link_libraries(depthprocessing ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
