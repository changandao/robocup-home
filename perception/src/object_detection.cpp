#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <all_msgs/send_flags.h>
#include <std_srvs/Empty.h>

int flag = 0;

class Object_detection
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

	image_transport::Subscriber sub_img_raw, sub_img;

  ros::Publisher pose_pub, pub_pt;
  ros::Subscriber sub_camInfo;

  tf::TransformListener _tfListener;

  int xmin, xmax, ymin, ymax;
  float fx, fy, cx, cy;
  bool dtok, pcok;

	void imageCallback(const sensor_msgs::ImageConstPtr& img);
  void processDepth(const sensor_msgs::ImageConstPtr& _img);
  void depthCameraInfo(const sensor_msgs::CameraInfoConstPtr& _info);
  cv::Vec3f getDepth(const cv::Mat& depthImage, int x, int y, float cx, float cy, float fx, float fy);

  pthread_mutex_t count_mutex;

public:
	Object_detection(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
	{
    dtok = false;
    pcok = false;

		image_transport::ImageTransport it(nh_);
    sub_camInfo = nh_.subscribe("/xtion/rgb/camera_info", 10, &Object_detection::depthCameraInfo, this);
		sub_img = it.subscribe("/xtion/rgb/image_rect_color", 10, &Object_detection::imageCallback, this);
    sub_img_raw = it.subscribe("/xtion/depth_registered/image_raw", 10, &Object_detection::processDepth, this);

    pose_pub = priv_nh_.advertise<geometry_msgs::PoseStamped>("Pick_Pose", 10);

    pub_pt = nh_.advertise<geometry_msgs::PointStamped>("/God_watcher/Point3D", 1);

    count_mutex = PTHREAD_MUTEX_INITIALIZER;

    ROS_INFO("Object detection initialized...");
	}

	~Object_detection()
  {
    ROS_INFO("Object detection finished...");
  }
};

void Object_detection::depthCameraInfo(const sensor_msgs::CameraInfoConstPtr& _info)
{
    fx = _info->K.at(0);
    fy = _info->K.at(4);
    cx = _info->K.at(2);
    cy = _info->K.at(5);
    pcok = true;
}

cv::Vec3f Object_detection::getDepth(const cv::Mat & depthImage, int x, int y, float cx, float cy, float fx, float fy)
{
    if(!(x >=0 && x<depthImage.cols && y >=0 && y<depthImage.rows))
    {
        ROS_WARN("Point must be inside the image (x=%d, y=%d), image size=(%d,%d)", x, y, depthImage.cols, depthImage.rows);
        return cv::Vec3f(
                    std::numeric_limits<float>::quiet_NaN (),
                    std::numeric_limits<float>::quiet_NaN (),
                    std::numeric_limits<float>::quiet_NaN ());
    }

    cv::Vec3f pt;

    // Use correct principal point from calibration
    float center_x = cx; //cameraInfo.K.at(2)
    float center_y = cy; //cameraInfo.K.at(5)

    bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    float unit_scaling = isInMM?0.001f:1.0f;
    float constant_x = unit_scaling / fx; //cameraInfo.K.at(0)
    float constant_y = unit_scaling / fy; //cameraInfo.K.at(4)
    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    float depth;
    bool isValid;
    if(isInMM)
    {
        depth = (float)depthImage.at<uint16_t>(y,x);
        isValid = depth != 0.0f;
    }
    else
    {
        depth = depthImage.at<float>(y,x);
        isValid = std::isfinite(depth);
    }

    // Check for invalid measurements
    if (!isValid)
    {
        pt[0] = pt[1] = pt[2] = bad_point;
    }
    else
    {
        // Fill in XYZ
        pt[0] = (float(x) - center_x) * depth * constant_x;
        pt[1] = (float(y) - center_y) * depth * constant_y;
        pt[2] = depth*unit_scaling;

    }
    return pt;
}

void Object_detection::imageCallback(const sensor_msgs::ImageConstPtr& img)
{
	cv::Mat src_rgb = cv_bridge::toCvShare(img, "bgr8")->image;
  // cv::imwrite("/home/atHomeSS18/Documents/color_segmentation/img.jpg", src_rgb);
	cv::Mat src_hsv;
    cv::cvtColor(src_rgb, src_hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2;
    cv::inRange(src_hsv, cv::Scalar(0, 70, 50), cv::Scalar(15, 255, 255), mask1);
    cv::inRange(src_hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2);

    cv::Mat mask = mask1 | mask2;
    cv::blur(mask, mask, cv::Size(3,3));

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
  	std::vector<cv::Rect> boundRect;

    cv::Rect rect;

  	for( uint i = 0; i < contours.size(); i++ )
  	{
    	cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 10, true );
      rect = cv::boundingRect( cv::Mat(contours_poly[i]) );
      if(rect.width > 70 && rect.height > 70)
      {
        boundRect.push_back(rect);
      }
  	}

  	for(uint i = 0; i < boundRect.size(); i++)
  	{
  		cv::Scalar color = cv::Scalar(50, 50, 50);
  		// tl: left upper point, br: right lower point
  		cv::rectangle(src_rgb, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
  	}

  	cv::imshow("Segmented", src_rgb);
    cv::waitKey(1);

    cv::Rect box;
    int size = 0;

    if(boundRect.size() == 0)
    {
      ROS_INFO("Red Bag is not found..");
      dtok = false;
    }
    else
    {
      for( uint i = 0; i < boundRect.size(); i++ )
      {
        if(boundRect[i].height * boundRect[i].width > size)
        {
          size = boundRect[i].height * boundRect[i].width;
          box = boundRect[i];
        }
      }
    
      xmin = (int)box.tl().x;
      xmax = (int)box.br().x;
      ymin = (int)box.tl().y;
      ymax = (int)box.br().y;
      dtok = true;
    }
}

void Object_detection::processDepth(const sensor_msgs::ImageConstPtr& _img)
{
  pthread_mutex_lock( &this->count_mutex );
  cv_bridge::CvImagePtr in_imgPtr = cv_bridge::toCvCopy(_img, sensor_msgs::image_encodings::TYPE_32FC1);
  if(dtok && pcok)
  {
    cv::Vec3f point3D = getDepth(in_imgPtr->image, (xmax+xmin)/2, (ymax+ymin)/2 , cx, cy, fx, fy);
    geometry_msgs::PoseStamped poseMsg;

    geometry_msgs::PointStamped Point_in, Point_out;
    Point_in.point.x = point3D[0];
    Point_in.point.y = point3D[1];
    Point_in.point.z = point3D[2];

    Point_in.header.frame_id = "xtion_rgb_optical_frame";
    // Point_in.header.stamp = ros::Time(0);//pc->header.stamp;
    Point_in.header.stamp = ros::Time(0);
    pub_pt.publish(Point_in);

    _tfListener.transformPoint("/base_footprint", Point_in, Point_out);

    // ROS_INFO("tf information found.");
    poseMsg.pose.position = Point_out.point;
    poseMsg.pose.orientation.x = 0;
    poseMsg.pose.orientation.y = 0;
    poseMsg.pose.orientation.z = 0;
    poseMsg.pose.orientation.w = 1;
    poseMsg.header = Point_out.header;
    poseMsg.header.stamp = ros::Time(0);

    pose_pub.publish(poseMsg);
  }
  pthread_mutex_unlock( &this->count_mutex );
}

bool getOrder(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{

    flag = 1;

    return true;

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "object_detection");
	ros::NodeHandle nh;
  ros::Rate r(5);

  ros::ServiceServer service = nh.advertiseService("/tiago/pick_object", getOrder);

  while(ros::ok())
  {
    if(flag) break;
    ros::spinOnce();

    r.sleep();
  }

  Object_detection object_detection(nh);

	ros::spin();
	
	return 0;
}
