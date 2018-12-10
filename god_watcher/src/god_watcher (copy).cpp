#include <god_watcher.h>

God_watcher::God_watcher()
{
    FLAG_nav_catch = false;
    FLAG_manipulation_pick = false;
    FLAG_nav_put = false;

    ROS_INFO("God is comming now...");
}

God_watcher::~God_watcher()
{
    ROS_INFO("God has gone...");
}

bool God_watcher::set_flag_catch(all_msgs::send_flags::Request &req, all_msgs::send_flags::Response &res)
{
    if(req.flag){
        FLAG_nav_catch = bool(req.flag);
        res.reply = 1;
        ROS_INFO("I have get the signal");\
        return true;
    }
    else return false;
}

bool God_watcher::set_flag_put(all_msgs::send_flags::Request &req, all_msgs::send_flags::Response &res)
{
    if(req.flag){
        FLAG_nav_put = req.flag;
        res.reply = 1;
        return true;
    }
    else return false;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "god_watcher");

    ros::NodeHandle n;
    God_watcher godwatcher;
    ros::Rate r(10);

    ros::ServiceClient client_navigation_catch = n.serviceClient<all_msgs::send_flags>("/tiago/catch");
    ros::ServiceClient client_navigation_turn = n.serviceClient<all_msgs::send_flags>("/tiago/turn");
    ros::ServiceClient client_navigation_put = n.serviceClient<all_msgs::send_flags>("/tiago/put");

    ros::ServiceClient client_getPoints = n.serviceClient<all_msgs::send_flags>("/tiago/from_2d_to_3d");
    ros::ServiceClient client_sending_pose = n.serviceClient<all_msgs::send_flags>("/tiago/send_place_msg");

    ros::ServiceClient client_pick = n.serviceClient<all_msgs::send_flags>("/tiago/pick");
    ros::ServiceClient client_lower_torso = n.serviceClient<all_msgs::send_flags>("/tiago/lower_torso");
    ros::ServiceClient client_place = n.serviceClient<all_msgs::send_flags>("/tiago/place");
    ros::ServiceClient client_readygo = n.serviceClient<all_msgs::send_flags>("/tiago/readygo");

    std_srvs::Empty manipulation;
    all_msgs::send_flags flag_msg;
    ROS_INFO("god is watching you...");

    ros::ServiceServer service_flag_catch = n.advertiseService("/flag/catch", &God_watcher::set_flag_catch, &godwatcher);
    ros::ServiceServer service_flag_put = n.advertiseService("/flag/put", &God_watcher::set_flag_put, &godwatcher);

    flag_msg.request.flag = 1;
    while(ros::ok())
    {
        if(godwatcher.FLAG_nav_catch) break;
        //ROS_INFO("localizing....");
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("localized");
    if(client_getPoints.call(flag_msg)) ROS_INFO("now we are in front of the table...");
    else ROS_INFO("no idea...");

    ROS_INFO("starting navigation...");

    if(client_navigation_catch.call(flag_msg)) {
        godwatcher.FLAG_manipulation_pick = true;
        ROS_INFO("now we are in front of the shelf...");
    }
    else ROS_INFO("no idea...");


    while(ros::ok())
    {
        if(godwatcher.FLAG_manipulation_pick) break;
    }




    if(client_readygo.call(flag_msg)) ROS_INFO("now we are to go...");
    else ROS_INFO("no idea...");

    if(client_pick.call(flag_msg)) ROS_INFO("it seems that we have got the object...");
    else ROS_INFO("no idea...");
//    client_pick.call(manipulation);
//    ROS_INFO("it seems that we have got the object...");

    if(client_navigation_turn.call(flag_msg)) ROS_INFO("We have turned 90 degrees...");
    else ROS_INFO("no idea...");
    
//    if(client_lower_torso.call(flag_msg)) ROS_INFO("We have lower the torso...");
//    else ROS_INFO("no idea...");

    //client_navigation_put.call(flag_msg);
    if(client_navigation_put.call(flag_msg)) ROS_INFO("now we are in front of the table...");
    else ROS_INFO("no idea...");

    if(client_sending_pose.call(flag_msg)) ROS_INFO("The position have been sent...");
    else ROS_INFO("no idea...");

    if(client_place.call(flag_msg)) ROS_INFO("now we have put the object on the table...");
    else ROS_INFO("no idea...");
//    client_place.call(manipulation);
//    ROS_INFO("place the object on the table");

    ros::spin();
    return 0;
}
