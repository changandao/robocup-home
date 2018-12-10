#ifndef MYTRACKERKCF_H
#define MYTRACKERKCF_H

#include <iostream>
#include <cstring>
#include <stdio.h>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <all_msgs/Box.h>
#include <all_msgs/send_box.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/tracking.hpp>

using namespace std;
using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";
//RNG rng(12345);

class MyTrackerKCF{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber box_sub;
    ros::ServiceServer service_box;

    Ptr<TrackerKCF> tracker;
    Rect2d roi;
    Mat frame;
    cv::Mat mIn;
    string desired_id;

    int flag_track;
    int c;
    int c_sub;

    //image_transport::Publisher image_pub_;
public:
    MyTrackerKCF();
    ~MyTrackerKCF();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    //void boxsubCb(const all_msgs::BoxConstPtr& msg);
    bool boxCb(all_msgs::send_box::Request &req, all_msgs::send_box::Response &res);

    //  void showImage(string title, const cv::Mat &mat);

};

#endif
