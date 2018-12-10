#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <std_srvs/Empty.h>
//#include <all_msgs/send_flags.h>

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

class Head_tracking
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

	std::string camera_frame;
	std::string point_topic;
	float fx,fy,cx,cy;
<<<<<<< HEAD
	bool pcok;
=======
	bool pcok;\
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e

	int control_flag;

	cv::Vec3f Point;

	PointHeadClientPtr pointHeadClient;

	ros::Subscriber sub_pt, sub_info;
<<<<<<< HEAD
//	ros::ServiceServer control_service;
=======
	ros::ServiceServer control_service;
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e

	void HeadCallback(const geometry_msgs::PointStampedConstPtr& _pt);
	void CameraInfo(const sensor_msgs::CameraInfoConstPtr& _info);
	void createPointHeadClient(PointHeadClientPtr& actionClient);
	bool head_control(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
public:
	Head_tracking(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
	{
		camera_frame = "/xtion_rgb_optical_frame";
		point_topic = "/God_watcher/Point3D";
		pcok = false;
		Point[0] = 0.0;
		Point[0] = 0.0;
		Point[0] = 0.0;
		control_flag = 0;
		sub_info = nh_.subscribe("/xtion/rgb/camera_info", 1, &Head_tracking::CameraInfo, this);
		sub_pt = nh_.subscribe(point_topic, 1, &Head_tracking::HeadCallback, this);
<<<<<<< HEAD
//                control_service = nh_.advertiseService("/head_tracking/control", &Head_tracking::head_control, this);
=======
		control_service = nh_.advertiseService("/head_tracking/control", &Head_tracking::head_control, this);
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
		ROS_INFO("head tracking started..");
	}
	~Head_tracking() {};
};

bool Head_tracking::head_control(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	control_flag++;
	return true;
}

void Head_tracking::CameraInfo(const sensor_msgs::CameraInfoConstPtr& _info)
{
	fx = _info->K.at(0);
	fy = _info->K.at(4);
	cx = _info->K.at(2);
	cy = _info->K.at(5);
	pcok = true;
}

void Head_tracking::createPointHeadClient(PointHeadClientPtr& actionClient)
{
	actionClient.reset( new PointHeadClient("/head_controller/point_head_action") );
	int iterations = 0, max_iterations = 3;
	// Wait for head controller action server to come up
	while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
	{
		ROS_DEBUG("Waiting for the point_head_action server to come up");
		++iterations;
	}

	if ( iterations == max_iterations )
		throw std::runtime_error("Error in createPointHeadClient: head controller action server not available");
}

void Head_tracking::HeadCallback(const geometry_msgs::PointStampedConstPtr& _pt)
{
<<<<<<< HEAD
        if( std::pow(_pt->point.x - Point[0], 2) + std::pow(_pt->point.y - Point[1], 2) > 0.01  && _pt->point.z == _pt->point.z &&_pt->point.x == _pt->point.x && _pt->point.z <8 && _pt->point.x < 8)
=======
        if( control_flag%2==1 && (std::pow(_pt->point.x - Point[0], 2) + std::pow(_pt->point.y - Point[1], 2) > 0.0225) )
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
	{
		Point[0] = _pt->point.x;
		Point[1] = _pt->point.y;
		Point[2] = _pt->point.z;

		createPointHeadClient( pointHeadClient );
		control_msgs::PointHeadGoal goal;
		goal.target = *_pt;
<<<<<<< HEAD
                goal.target.point.y -= 0.2;
=======
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
		goal.pointing_frame = camera_frame;
		goal.pointing_axis.x = 0.0;
		goal.pointing_axis.y = 0.0;
		goal.pointing_axis.z = 1.0;
<<<<<<< HEAD
                goal.min_duration = ros::Duration(0.75);
                goal.max_velocity = 0.3;
=======
		goal.min_duration = ros::Duration(1.0);
		goal.max_velocity = 0.25;
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e

		pointHeadClient->sendGoal(goal);

	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "head_tracking");

	ros::NodeHandle nh;
	Head_tracking ht(nh);

	ros::spin();

	return 0;
}
