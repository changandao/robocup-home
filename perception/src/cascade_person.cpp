#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

class Cascade_person
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

	image_transport::ImageTransport it_;
	image_transport::Subscriber sub_img;
	image_transport::Publisher pub_img;

	string head_detect;
	string upper_body_detect;

	CascadeClassifier head_cas;
	CascadeClassifier upper_body_cas;

	void ImageCallback(const sensor_msgs::ImageConstPtr& _img);
public:
	Cascade_person(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"), it_(priv_nh_)
	{
		head_detect = "/home/atHomeSS18/test_workspace/src/cascade_file/cascadeH5.xml";
		upper_body_detect = "/home/atHomeSS18/test_workspace/src/cascade_file/haarcascade_upperbody.xml";

		bool head_load = head_cas.load(head_detect);
		bool upbody_load = upper_body_cas.load(upper_body_detect);

		sub_img = it_.subscribe("/xtion/rgb/image_rect_color", 10, &Cascade_person::ImageCallback, this);

		namedWindow("Detected Image");
		ROS_INFO("cascade detecting...");
	}
	~Cascade_person() {};
	
};

Cascade_person::ImageCallback(const sensor_msgs::ImageConstPtr& _img)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(_img, sensor_msgs::image_encodings::BGR8);
	vector<Rect> head;
	vector<Rect> upbody;

	Mat src_gray;
	cvtColor(cv_ptr->image, CV_BGR2GRAY);
	
}