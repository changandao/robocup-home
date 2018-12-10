#ifndef TFPoints_H
#define TFPoints_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <all_msgs/Object.h>
#include <all_msgs/ObjectArray.h>

class TFPoints {
public:
	TFPoints(ros::NodeHandle nh);

	// Virtual destructor
	~TFPoints();
	ros::Time timer;
	virtual void process(const all_msgs::ObjectArrayConstPtr & detected_objects);

private:
	ros::NodeHandle nh_, priv_nh_;
	ros::Subscriber sub_originPoints;
	ros::Publisher pub_objs;
	tf::TransformListener listener_;
};

#endif
