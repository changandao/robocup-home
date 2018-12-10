#include <ros/ros.h>
#include <betracker_lib/ukf.h>
#include <betracker_lib/TFPoints.h>


int main(int argc, char **argv){
        ros::init(argc, argv, "tracker_ukf");
        ros::NodeHandle nh;
        TFPoints tfpoints(nh);
	tracking::UnscentedKF tracker(nh);
	ros::spin();

	return 0;
}
