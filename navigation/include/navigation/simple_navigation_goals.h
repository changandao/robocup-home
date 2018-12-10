#ifndef SIMPLE_NAVIGATION_GOALS_H
#define SIMPLE_NAVIGATION_GOALS_H


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Quaternion.h>
#include <all_msgs/Box.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <all_msgs/send_pos.h>//geometry point
#include <all_msgs/send_pose.h>//geometry pose

// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <iostream>

#include <std_msgs/String.h>
#include <all_msgs/send_flags.h>
#include <all_msgs/Object.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace Eigen;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

char choose();
//bool moveGoal(std::string frame_id, double xGoal, double yGoal, double rzGoal, double rwGoal);
//void move_arm(double *arm_pose);

class Navigation
{
private:
    //ros::NodeHandle nh_;
    //ros::NodeHandle priv_nh_;
    bool flag_catch, flag_put;
    bool flag_right_pose;
    double x, y, rz, rw;
    
    ros::NodeHandle nh_, private_nh_;
    
    float yaw;
    float distance;
    Vector3d tiago_desired_g;
    Vector3d tiagoPose_g;
    Vector3d target_cam_pos;
    tf::TransformListener listener_;
    tf::Quaternion q;
    Quaterniond q_orig;
    Quaterniond q_obj;


    
    std_srvs::Empty empty_srv;
    ros::ServiceClient localization_client;
    ros::Publisher track_point_pub_;
    
    pthread_mutex_t count_mutex;
    

public:
    Navigation(ros::NodeHandle nh);
    ~Navigation();

    void getCurPose(const all_msgs::ObjectConstPtr& obj_msg);
    //bool follow_srv(all_msgs::send_pos::Request &req, all_msgs::send_pos::Response &res);
    bool follow(all_msgs::send_flags::Request & req, all_msgs::send_flags::Response & res);
    //void getTiagoPose(const nav_msgs::Odometry::ConstPtr &msg);
    bool localization(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    ros::Time timer;
    
    bool setflag();
    void set_cords(double re_x, double re_y, double re_rz, double re_rw);
    bool moveGoal(std::string frame_id, double xGoal, double yGoal, double rzGoal, double rwGoal);
    bool moveRelative(double set_time, double v, double w);
    //bool is_nan(int d);

};

#endif
