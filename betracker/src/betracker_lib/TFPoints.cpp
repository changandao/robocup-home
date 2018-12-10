#include <betracker_lib/TFPoints.h>


void TFPoints::process(const all_msgs::ObjectArrayConstPtr & detected_objects)
{
	all_msgs::ObjectArray objects_list;
	objects_list.header = detected_objects->header;
	all_msgs::Object tracking_object;
	for (int i = 0; i < detected_objects->list.size(); ++i)
	{
		//ROS_INFO_STREAM("THIS IS SOMETHINF "<<detected_objects->list[i].id);
		tracking_object.id = detected_objects->list[i].id;
		tracking_object.heading = detected_objects->list[i].heading;
		tracking_object.velocity = detected_objects->list[i].velocity;
		tracking_object.width = detected_objects->list[i].width;
		tracking_object.length = detected_objects->list[i].length;
		tracking_object.semantic_confidence = detected_objects->list[i].semantic_confidence;
		tracking_object.semantic_id = detected_objects->list[i].semantic_id;
		tracking_object.r = detected_objects->list[i].r;
		tracking_object.g = detected_objects->list[i].g;
		tracking_object.b = detected_objects->list[i].b;
		tracking_object.is_track = detected_objects->list[i].is_track;
		tracking_object.is_target = detected_objects->list[i].is_target;

		tracking_object.cam_pose.header.stamp = timer;
		tracking_object.cam_pose.header.frame_id = "xtion_rgb_optical_frame";
		tracking_object.cam_pose = detected_objects->list[i].cam_pose;
//		tracking_object.cam_pose.point.x = detected_objects->list[i].cam_pose.y;
		//ros::Time now = ros::Time::now();
		//ros::Time past = now - ros::Duration(531.0);
		if (tracking_object.cam_pose.point.z == tracking_object.cam_pose.point.z && tracking_object.cam_pose.point.z < 5)
		{
			try {
				listener_.transformPoint("map",
				                         tracking_object.cam_pose,
				                         tracking_object.world_pose);

			} catch (tf::TransformException& ex)
			{
				ROS_ERROR("Received an exception trying to transform a point from "
				          "\"camero link\" to \"map\": %s", ex.what());
			}

			if(tracking_object.world_pose.point.z == tracking_object.world_pose.point.z)
				objects_list.list.push_back(tracking_object);
		}


	}
	pub_objs.publish(objects_list);
}


TFPoints::TFPoints(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
{
	ROS_INFO("Points transformer begins...");
	sub_originPoints = nh_.subscribe("/depthprocessing/ObjectArray", 2, &TFPoints::process, this);
	pub_objs = nh_.advertise<all_msgs::ObjectArray>("/object_sending/TransformdObjectArray", 2);

}

TFPoints::~TFPoints() {}
