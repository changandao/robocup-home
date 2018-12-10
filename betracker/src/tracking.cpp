#include "tracking.h"


MyTrackerKCF::MyTrackerKCF()
    : it_(nh_), roi(0,0,0,0)
{

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/xtion/rgb/image_rect_color", 30,
                               &MyTrackerKCF::imageCb, this);
    //ball_pub_ = nh_.advertise<geometry_msgs::Point>("/image_processing/ball",1);
    //box_sub = nh_.subscribe<all_msgs::Box>("/follow_me/Box",10, &MyTrackerKCF::boxsubCb, this);
    service_box = nh_.advertiseService("/follow_me/Box", &MyTrackerKCF::boxCb, this);
    flag_track = 0;
    c = 0;
    c_sub = 0;
    tracker = TrackerKCF::create();
    desired_id = "person";

    namedWindow(OPENCV_WINDOW);
    startWindowThread();
}


MyTrackerKCF::~MyTrackerKCF()
{
    destroyWindow(OPENCV_WINDOW);
}


bool MyTrackerKCF::boxCb(all_msgs::send_box::Request &req, all_msgs::send_box::Response &res)
{
    
    ROS_INFO("I have find the guy");
    flag_track = 1;
    roi.x = double(req.x);
    roi.y= double(req.y);
    roi.width = double(req.width); 
    roi.height = double(req.height);
    res.reply = 1;
    ROS_INFO_STREAM("fist show:  "<<"bb's object: "<< req.id <<"bb's x: "<<roi.x<<" bb's y: "<<roi.y<<" bb's x: "<<roi.width<<" bb's x: "<<roi.height);
    return true;
}


// void MyTrackerKCF::boxsubCb(const all_msgs::BoxConstPtr& msg)
// {
//     if(msg->id == desired_id)
//     {
//         c_sub++;
//         if(c_sub>5)
//         {
//             ROS_INFO("I have find the guy");
//             flag_track = 1;
//             roi.x = double(msg->x);
//             roi.x = double(msg->y);
//             roi.width = double(msg->width); 
//             roi.height = double(msg->height);
//             ROS_INFO_STREAM("fist show:  "<<"bb's object: "<< msg->id <<"bb's x: "<<msg->x<<" bb's y: "
//                 <<msg->y<<" bb's x: "<<msg->width<<" bb's x: "<<msg->height);
//         }
        
//     }
    
// }

void MyTrackerKCF::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mIn = cv_ptr->image;
    frame = mIn;
    
    if (flag_track == 1)
    {

        c++;
        if (roi.width == 0 || roi.height == 0)
            ROS_INFO("not valid roi");
        if(c <5) 
        {
            tracker->init(frame, roi);
            ROS_INFO("Start the tracking process");
        }
        else{
            ROS_INFO("tracking...");
            tracker->update(frame, roi);
            ROS_INFO("tracking done...");

            rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
            namedWindow( "tracked", CV_WINDOW_AUTOSIZE );
            imshow("tracker", frame);
        }
        if(c==10) c--;
        

        // draw the tracked object
        
    }
    // else if(flag_track == 1)
    // {
    //     tracker->update(frame, roi);

    //     // draw the tracked object
    //     rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
    //     namedWindow( "tracked", CV_WINDOW_AUTOSIZE );
    //     imshow("tracker", frame);
    // }
    namedWindow( "origin", CV_WINDOW_AUTOSIZE );
    imshow("origin", mIn);

    waitKey(5);
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tracking_KCF");
    MyTrackerKCF MyTrackerKCF;
    ros::spin();


    return 0;
}
