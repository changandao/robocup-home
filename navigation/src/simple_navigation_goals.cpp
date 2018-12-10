#include <navigation/simple_navigation_goals.h>

Navigation::Navigation(ros::NodeHandle nh): nh_(nh),
    private_nh_("~")
{
    x = 0;
    y = 0;
    rz = 0;
    rw = 0;
    flag_catch = false;
    flag_put = false;
    flag_right_pose = false;
    yaw = 0;
    distance = 0.7;

    tiagoPose_g << 0., 0., 0.;
    tiago_desired_g << 0., 0., 0.;
    target_cam_pos << 0., 0., 0.;
    track_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/navigation/tracking_point", 2);
    localization_client = nh_.serviceClient<std_srvs::Empty>("/global_localization");
    count_mutex = PTHREAD_MUTEX_INITIALIZER;
    ROS_INFO("god is watching you...");

}


Navigation::~Navigation()
{

    ROS_INFO("Now god has gone...");
}

void Navigation::set_cords(double re_x, double re_y, double re_rz, double re_rw)
{
    x = re_x;
    y = re_y;
    rz = re_rz;
    rw = re_rw;
}


char choose() {
    char choice = 'q';
    std::cout << "|------------------------------------------------|" << std::endl;
    std::cout << "|PRESSE A KEY:" << std::endl;
    std::cout << "|'y': yes (and continue for navigation)           " << std::endl;
    std::cout << "|'n': no (rotate another circle to continue localizing)" << std::endl;
    std::cout << "|'r': retry (relocalize itself, also rerun amcl)  " << std::endl;
    std::cout << "|'q': Quit " << std::endl;
    std::cout << "|------------------------------------------------|" << std::endl;
    std::cout << "|yes or no or retry?: ";
    std::cin >> choice;
    //client_confirm = nh.n.serviceClient<std_srvs::Empty>("/global_localization");

    return choice;
}


bool Navigation::moveRelative(double set_time, double v, double w)
{
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 100);
    geometry_msgs::Twist msg;
    msg.linear.x = v;
    msg.angular.z = w;
    ros::Rate r(60); // 10 hz
    int c = 0;
    set_time = set_time;
    while (ros::ok())
    {
        c++;
        if (c > set_time) break;
        pub.publish(msg);
        //ROS_INFO("Localizing");
        ros::spinOnce();
        r.sleep();
    }
    return true;
}


bool Navigation::moveGoal(std::string frame_id, double xGoal, double yGoal, double rzGoal, double rwGoal)
{
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(10.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.header.stamp = ros::Time::now();



    goal.target_pose.pose.position.x = xGoal;
    goal.target_pose.pose.position.y = yGoal;
    goal.target_pose.pose.position.z = 0.;
    goal.target_pose.pose.orientation.x = 0.;
    goal.target_pose.pose.orientation.y = 0.;
    goal.target_pose.pose.orientation.z = rzGoal;
    goal.target_pose.pose.orientation.w = rwGoal;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("You have seccessfully reached the goal position");
        return true;
    }
    else {
        ROS_INFO("The base failed to move to the goal position for some reason");
        return false;
    }

}



bool Navigation::localization(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    localization_client.call(empty_srv);
    bool rotatet = moveRelative(1516, 0.01, 0.5);

    if (rotatet) return true;
    else return false;

}


void Navigation::getCurPose(const all_msgs::ObjectConstPtr& obj_msg)
{
    pthread_mutex_lock( &this->count_mutex );
    tf::StampedTransform transform;
    tiago_desired_g(0) = obj_msg->world_pose.point.x;
    tiago_desired_g(1) = obj_msg->world_pose.point.y;
    //tiago_desired_g[2] = obj_msg.world_pose.point.z;
    //pose.orientation = tf::createQuaternionMsgFromYaw(angle);
//camera_point.header.stamp = timer;
   //     camera_point.header.frame_id = "xtion_rgb_optical_frame";

    try {
        ros::Time now = timer;//ros::Time::now();
        //ros::Time past = now - ros::Duration(541.2);
        // listener_.waitForTransform("/base_link", past,
        //                            "/map", now,
        //                            "/map", ros::Duration(0.0));
        // listener_.lookupTransform("/base_link", past,
        //                           "/map", now,
        //                           "/map", transform);
        listener_.waitForTransform("/map","/base_link", timer,
                                    ros::Duration(0.0));
        listener_.lookupTransform("/map","/base_link", timer, transform);
    }
    catch (tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point from"
                  "\"base\" to \"map\": %s", ex.what());
    }
    tiagoPose_g(0) = transform.getOrigin().x();
    tiagoPose_g(1) = transform.getOrigin().y();
    //####get the yaw from front of the robot to the target
    q = transform.getRotation();
    tiago_desired_g(2) = tf::getYaw(q);

    //yaw = atan2(tiago_desired_g(0) - tiagoPose_g(0), tiago_desired_g(1) - tiagoPose_g(1)); //+tiago_desired_g(2);
    yaw = atan2(tiago_desired_g(1) - tiagoPose_g(1), tiago_desired_g(0) - tiagoPose_g(0)); 
    q.setRPY(0., 0., yaw);

    float fern = sqrt((tiago_desired_g(0) - tiagoPose_g(0))*(tiago_desired_g(0) - tiagoPose_g(0)) + 
        (tiago_desired_g(1) - tiagoPose_g(1))*(tiago_desired_g(1) - tiagoPose_g(1)));
    if (fern != 0.0)
    {
        tiago_desired_g(0) = distance/fern * tiagoPose_g(0) + (fern - distance)/ fern * tiago_desired_g(0);// + tiagoPose_g(0);
        tiago_desired_g(1) = distance/fern * tiagoPose_g(1) + (fern - distance)/ fern * tiago_desired_g(1);;// + tiagoPose_g(1);
    }
    else 
        ROS_INFO("There is someshing wrong with the given point");
    
//    geometry_msgs::PointStamped pose_msg;
//    pose_msg.point.x = tiago_desired_g(0);
//    pose_msg.point.y = tiago_desired_g(1);
//    pose_msg.point.z = yaw;

//    // pose_msg.pose.orientation.x = q.x();
//    // pose_msg.pose.orientation.y = q.y();
//    // pose_msg.pose.orientation.z = q.z();
//    // pose_msg.pose.orientation.w = q.w();

//    track_point_pub_.publish(pose_msg);

    pthread_mutex_unlock( &this->count_mutex );
}

//bool Navigation::follow(all_msgs::send_pos::Request & req, all_msgs::send_pos::Response & res) {

////    set_cords();
////    bool backward=Navigation::moveGoal("map", x, y,rz,rw);
//    //pthread_mutex_lock( &this->count_mutex );
//    ROS_INFO("Now we gonna to follow");

//    if (req.is_target)
//    {
//        tiago_desired_g(0) = req.world_pose.point.x - 1.5 * cos(req.world_pose.point.z);// + tiagoPose_g(0);
//        tiago_desired_g(1) = req.world_pose.point.y - 1.5 * sin(req.world_pose.point.z);// + tiagoPose_g(1);
//        q.setRPY(0, 0, req.world_pose.point.z);
//        ROS_INFO_STREAM("X: " << tiago_desired_g(0) << "Y: " << tiago_desired_g(1) << "Z: " << q.z() << "W: " << q.w());
//        set_cords(tiago_desired_g(0), tiago_desired_g(1), q.z(), q.w());
//        bool flag_onpose = moveGoal("map", x, y, rz, rw);
//        ROS_INFO("right on the place...");
//        if (flag_onpose)
//            res.flag = 1;
//        else res.flag = 0;
//        return true;
//    }
//    else return false;
//    //pthread_mutex_unlock( &this->count_mutex );
//}



bool Navigation::follow(all_msgs::send_flags::Request & req, all_msgs::send_flags::Response & res) {

//    set_cords();
//    bool backward=Navigation::moveGoal("map", x, y,rz,rw);
    //pthread_mutex_lock( &this->count_mutex );
<<<<<<< HEAD
    ROS_INFO("Now I gonna to follow");
    if(req.flag == 1){
=======
    ROS_INFO("Now we gonna to follow");

    if (req.is_target)
    {
        tiago_desired_g(0) = req.world_pose.point.x - 1.5 * cos(req.world_pose.point.z);// + tiagoPose_g(0);
        tiago_desired_g(1) = req.world_pose.point.y - 1.5 * cos(req.world_pose.point.z);// + tiagoPose_g(1);
        q.setRPY(0, 0, req.world_pose.point.z);
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
        ROS_INFO_STREAM("X: " << tiago_desired_g(0) << "Y: " << tiago_desired_g(1) << "Z: " << q.z() << "W: " << q.w());
        set_cords(tiago_desired_g(0), tiago_desired_g(1), q.z(), q.w());
        bool flag_onpose = moveGoal("map", x, y, rz, rw);
        ROS_INFO("right on the place...");
<<<<<<< HEAD
        if(flag_onpose)
            res.reply = 1;
        else 
            res.reply = 0;
=======
        if (flag_onpose)
            res.flag = 1;
        else res.flag = 0;
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
        return true;
    }
    // else
    //     return false;
        

    //pthread_mutex_unlock( &this->count_mutex );
}



bool Navigation::setflag() {
    return flag_right_pose;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_navigation_goals");

    ros::NodeHandle n;
    Navigation nav(n);
    ros::Duration(5).sleep();


    ///========================================= localization part ========================

    ros::Subscriber target_sub = n.subscribe("/tracking/target", 10, &Navigation::getCurPose, &nav);
    // ros::Subscriber sub = n.subscribe("/mobile_base_controller/odom", 100,
    //                                   &Navigation::getTiagoPose,
    //                                   &nav);
    ros::ServiceServer service_localization = n.advertiseService("/god_watcher/loc", &Navigation::localization, &nav);
    //ros::ServiceClient client_localized = n.serviceClient<all_msgs::send_flags>("/god_watcher/ready");
    // ros::ServiceClient put_client = n.serviceClient<all_msgs::send_flags>("/flag/put");


    ///========================================================================================
//    all_msgs::send_flags flag_msg;
//    flag_msg.request.flag = true;
//    client_localized.call(flag_msg);
    ros::ServiceServer service_follow = n.advertiseService("/navigation/follow", &Navigation::follow, &nav);


    ros::spin();
    return 0;

}
