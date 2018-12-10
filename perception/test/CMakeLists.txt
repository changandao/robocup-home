cmake_minimum_required(VERSION 2.8)
project(sift_test)

find_package(OpenCV REQUIRED)
add_executable(sift_test SIFT.cpp)
target_link_libraries(sift_test ${OpenCV_LIBS})