#ifndef GOD_WATCHER_H
#define GOD_WATCHER_H

/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <all_msgs/send_flags.h>
#include <std_srvs/Empty.h>
#include <all_msgs/Object.h>


class God_watcher{
private:
    //int FLAG_local;

//    bool FLAG_nav_c;

//    bool FLAG_percrption;

//    bool FLAG_manipulation;

    //ros::NodeHandle nh_;
    //ros::NodeHandle priv_nh_;
    bool isnan_(float fnan);


public:
    God_watcher();
    ~God_watcher();
    bool FLAG_locolized;
    bool FLAG_follow;
    bool set_locolized(all_msgs::send_flags::Request &req, all_msgs::send_flags::Response &res);
    void getCurPose(const all_msgs::ObjectConstPtr& obj_msg);

    
//    bool get_FLAG_nav_catch();
//    bool get_FLAG_nav_put();
};


#endif
