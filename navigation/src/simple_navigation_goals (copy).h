#ifndef SIMPLE_NAVIGATION_GOALS_H
#define SIMPLE_NAVIGATION_GOALS_H


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <all_msgs/Box.h>
// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <iostream>

#include <std_msgs/String.h>
#include <all_msgs/send_flags.h>

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

public:
    Navigation();
    ~Navigation();

    bool set_flag_catch(all_msgs::send_flags::Request &req, all_msgs::send_flags::Response &res);
    bool set_flag_put(all_msgs::send_flags::Request &req, all_msgs::send_flags::Response &res);
    //bool set_flag_end(perception_msgs::send_flags::Request &req, perception_msgs::send_flags::Response &res);

    bool turn(all_msgs::send_flags::Request &req, all_msgs::send_flags::Response &res);
    void bb_receive(const all_msgs::BoxConstPtr& bb_msg);
    
    bool setflag();
    void set_cords(double re_x, double re_y, double re_rz, double re_rw);
    bool moveGoal(std::string frame_id, double xGoal, double yGoal, double rzGoal, double rwGoal);
    bool moveRelative(double set_time, double v, double w);
    //bool is_nan(int d);

};

#endif
