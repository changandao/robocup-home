
// Include guard
#ifndef unscentedkf_H
#define unscentedkf_H

// Includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <all_msgs/Object.h>
#include <all_msgs/ObjectArray.h>
#include <geometry_msgs/PointStamped.h>

#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

// Namespaces
namespace tracking{

using namespace Eigen;

struct Parameter{

	float da_ped_dist_pos;
	float da_ped_dist_form;

	int tra_dim_z;
	int tra_dim_x;
	int tra_dim_x_aug;

	float tra_std_lidar_x;
	float tra_std_lidar_y;
	float tra_std_acc;
	float tra_std_yaw_rate;
	float tra_lambda;
	int tra_aging_bad;

	float tra_occ_factor;

	float p_init_x;
	float p_init_y;
	float p_init_v;
	float p_init_yaw;
	float p_init_yaw_rate;
};

struct History{

	int good_age;
	int bad_age;

};

struct Geometry{

	float width;
	float length;
        //float height;
        //float orientation;
};

struct Semantic{

	int id;
	std::string name;
	float confidence;
};

struct State{

	VectorXd x;
	float z;
	MatrixXd P;
	VectorXd x_aug;
	VectorXd P_aug;
	MatrixXd Xsig_pred;
};

struct Track{

	// Attributes
	int id;
	State sta;
	geometry_msgs::PointStamped cam_pose;
	Geometry geo;
	Semantic sem;
	History hist;
	int r;
	int g;
	int b;
	bool is_target;
};

class UnscentedKF{

public:

	// Default constructor
        UnscentedKF(ros::NodeHandle nh);

	// Virtual destructor
	virtual ~UnscentedKF();

        virtual void process(const all_msgs::ObjectArrayConstPtr & detected_objects);
        virtual bool control(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

protected:

	// Class member
	Parameter params_;
	bool isnan_(float fnan);

private:

	pthread_mutex_t count_mutex;

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Processing
	size_t pros_count;
	all_msgs::Object tmp_obj;
	bool is_initialized_;
	int track_id_counter_;
	int time_frame_;
	tf::TransformListener listener_;
	bool flag_if_ini;

	int control_count;

	int target_bad_age;
	int refind_target;

	// Visualization
	cv::RNG rng_;

	// UKF
	MatrixXd R_laser_;
	VectorXd weights_;
	std::vector<Track> tracks_;

	// Prediction
	double last_time_stamp_;

	// Subscriber
	ros::Subscriber list_detected_objects_sub_;

	// Service
	ros::ServiceServer ukf_control_service;

	// Publisher
	ros::Publisher list_tracked_objects_pub_;
        ros::Publisher tracked_target_pub_;
        ros::Publisher tracked_cam_pub_;
        ros::Publisher refind_target_pub_;

	void publishTracks(const std_msgs::Header & header);
    void publishTarget(const std_msgs::Header & header);
    void publishcamTarget(const std_msgs::Header & header);

    // CLient
    ros::ServiceClient refind_target_client_;

	// Class functions
	void Prediction(const double delta_t);
        void Update(const all_msgs::ObjectArrayConstPtr & detected_objects);
        	void clearTrack();
        void TrackManagement(const all_msgs::ObjectArrayConstPtr & detected_objects);
        void initTrack(const all_msgs::Object & obj);
        void printTrack(const Track & tr);
        void printTracks();


	// Data Association members
	std::vector<int> da_tracks;
	std::vector<int> da_objects;
	std::vector<int> da_targets;

	// Data Association functions
		float CalculatecamDistance(const Track & track);
        void GlobalNearestNeighbor(const all_msgs::ObjectArrayConstPtr & detected_objects);
        float CalculateDistance(const Track & track, const all_msgs::Object & object);
        float CalculateBoxMismatch(const Track & track, const all_msgs::Object & object);
	float CalculateEuclideanAndBoxOffset(const Track & track, 
                const all_msgs::Object & object);
};

} // namespace tracking

#endif // unscentedkf_H
