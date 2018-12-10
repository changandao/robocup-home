#include <god_watcher.h>

God_watcher::God_watcher()
{
    FLAG_locolized = false;
    FLAG_follow = false;

    ROS_INFO("God is comming now...");
}

God_watcher::~God_watcher()
{
    ROS_INFO("God has gone...");
}

bool God_watcher::set_locolized(all_msgs::send_flags::Request &req, all_msgs::send_flags::Response &res)
{
    if (req.flag) {
        FLAG_locolized = bool(req.flag);
        res.reply = 1;
        ROS_INFO("I have get ready to move");
        return true;
    }
    else return false;
}



bool God_watcher::isnan_(float fnan)
{
    return !(fnan == fnan);
}

void God_watcher::getCurPose(const all_msgs::ObjectConstPtr& obj_msg)
{
    if(!isnan_(obj_msg->world_pose.point.x))
        FLAG_follow = true;
    else
        FLAG_follow = false;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "god_watcher");

    ros::NodeHandle n;
    God_watcher godwatcher;
    ros::Rate r(10);

    ros::ServiceClient client_follow = n.serviceClient<all_msgs::send_flags>("/god_watcher/follow");
    ros::Subscriber target_sub = n.subscribe("/tracking/target", 10, &God_watcher::getCurPose, &godwatcher);


    //########################topic
    std_srvs::Empty manipulation;
    all_msgs::send_flags flag_msg;
    ROS_INFO("god is watching you...");

    ros::ServiceServer service_locolized = n.advertiseService("/god_watcher/ready", &God_watcher::set_locolized, &godwatcher);
    //ros::ServiceServer service_flag_put = n.advertiseService("/god_watcher/put", &God_watcher::set_flag_put, &godwatcher);

    flag_msg.request.flag = 1;
    while (ros::ok())
    {
        if (godwatcher.FLAG_locolized) break;
        //ROS_INFO("localizing....");
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("localized");


    ROS_INFO("starting navigation...");

    while (ros::ok())
    {
        if(godwatcher.FLAG_follow)
        {
            if (client_follow.call(flag_msg)) {
            //godwatcher.FLAG_manipulation_pick = true;
            ROS_INFO("now I am behind the person ...");
        }
            else ROS_INFO("I have lost");
        }
        
        ros::spinOnce();
        r.sleep();
    }




    // while(ros::ok())
    // {
    //     if(godwatcher.FLAG_manipulation_pick) break;
    // }



    ros::spin();
    return 0;
}
