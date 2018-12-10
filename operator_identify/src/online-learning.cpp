#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

class Online_learning
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

	image_transport::Subscriber sub_image;
	ros::Subscriber sub_box;

	std::string id = "person";
	int xmin = 0;
	int xmax = 0;
	int ymin = 0;
	int ymax = 0;
	int flag = 0;

	void BoxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bx);
	void imageCallback(const sensor_msgs::ImageConstPtr& img);

public:
	Online_learning(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
	{
		image_transport::ImageTransport it(nh_);
		sub_box = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &Online_learning::BoxCallback, this);
		sub_image = it.subscribe("/xtion/rgb/image_rect_color", 1, &Online_learning::imageCallback, this);

		ROS_INFO("Operator learning ready!");
	}
	~Online_learning();
	
};

void BoxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bx)
{
	float prob = 0.5;
	int candidate = -1;
	int count = 0;
	for(int i = 0; i < bx->bounding_boxes.size(); i++)
	{
		if(bx->bounding_boxes[i].Class == id && bx->bounding_boxes[i].probability > prob)
		{
			count++;
			candidate = i;
		}
	}
	if(count == 0)
	{
		ROS_INFO("God can't see legal person!");
		flag = 0;
	}
	else if(count > 1)
	{
		ROS_INFO("God is confusing!");
		flag = 0;
	}
	else
	{
		flag = 1;
		xmin = bx->bounding_boxes[candidate].xmin;
		xmax = bx->bounding_boxes[candidate].xmax;
		ymin = bx->bounding_boxes[candidate].ymin;
		ymax = bx->bounding_boxes[candidate].ymax;
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& img)
{
	while(flag = 1)
	{
		f
	}
}