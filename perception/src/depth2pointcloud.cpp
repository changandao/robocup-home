#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <all_msgs/Box.h>
#include <all_msgs/Boxes.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>

using namespace cv;
using namespace std;

class Depth2PointCloud
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    image_transport::Subscriber sub_image_raw, sub_image_rect;
    image_transport::Publisher pub_image;

    ros::Subscriber sub_camInfo, sub_box;
    ros::Publisher pub_pt;

    int xmin, ymin, height, width;
    int x_m, y_m;

    float fx, fy, cx, cy;
    float dd;
    bool pcok, bxok, mkok;

    Mat mask;
    // Rect rectangle;

    cv::Vec3f getDepth(const cv::Mat& depthImage, int x, int y, float cx, float cy, float fx, float fy);
    void BoxCallback(const all_msgs::BoxesConstPtr& bx);
    void depthCameraInfo(const sensor_msgs::CameraInfoConstPtr& _info);
    void processDepth(const sensor_msgs::ImageConstPtr& _img);
    void Imageprocessing(const sensor_msgs::ImageConstPtr& _img);

    pthread_mutex_t count_mutex;

public:
	Depth2PointCloud(ros::NodeHandle nh, float distance) : nh_(nh), priv_nh_("~")
	{
        pcok = false;
        bxok = false;
        mkok = false;

        dd = distance;
        image_transport::ImageTransport it(priv_nh_);
        sub_camInfo = nh_.subscribe("/xtion/rgb/camera_info", 10, &Depth2PointCloud::depthCameraInfo, this);
        sub_box = nh_.subscribe("/God_watcher/Boxes", 10, &Depth2PointCloud::BoxCallback, this);

        sub_image_raw = it.subscribe("/xtion/depth_registered/image_raw", 10, &Depth2PointCloud::processDepth, this);
        sub_image_rect = it.subscribe("/xtion/rgb/image_rect_color", 10, &Depth2PointCloud::Imageprocessing, this);

        pub_image = it.advertise("Segmented_Image", 10);

<<<<<<< HEAD
        pub_pt = nh_.advertise<geometry_msgs::PointStamped>("/God_watcher/Point3D", 1);
=======
        pub_pt = nh_.advertise<geometry_msgs::PointStamped>("/God_watcher/Point3D", 10);
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e

        count_mutex = PTHREAD_MUTEX_INITIALIZER;

        ROS_INFO("depth2pointcloud ready!");
	}

	~Depth2PointCloud() {};
	
};

cv::Vec3f Depth2PointCloud::getDepth(const cv::Mat & depthImage, int x, int y, float cx, float cy, float fx, float fy)
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

void Depth2PointCloud::depthCameraInfo(const sensor_msgs::CameraInfoConstPtr& _info)
{
    fx = _info->K.at(0);
    fy = _info->K.at(4);
    cx = _info->K.at(2);
    cy = _info->K.at(5);
    pcok = true;
}

void Depth2PointCloud::BoxCallback(const all_msgs::BoxesConstPtr& bx)
{
<<<<<<< HEAD
=======
    // xmin = bx->x;
    // ymin = bx->y;
    // height = bx->height;
    // width = bx->width;

    // x_m = xmin + width / 2;
    // y_m = ymin + width / 2;
    // bxok = true;
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
    int size = 0;
    int candidate = -1;
    for(int i=0; i < bx->Boxes.size(); i++)
    {
        if(bx->Boxes[i].height * bx->Boxes[i].width > size)
        {
            size = bx->Boxes[i].height * bx->Boxes[i].width;
            candidate = i;
        }
    }
<<<<<<< HEAD
    if(candidate >= 0)
    {
//        ROS_INFO("asdfasdfasdfas");
        xmin = bx->Boxes[candidate].x;
        ymin = bx->Boxes[candidate].y;
        width = bx->Boxes[candidate].width;
        height = bx->Boxes[candidate].height;
        x_m = bx->Boxes[candidate].x + bx->Boxes[candidate].width / 2;
        y_m = bx->Boxes[candidate].y + bx->Boxes[candidate].height / 2;
        bxok = true;
    }
=======
    x_m = bx->Boxes[candidate].x + bx->Boxes[candidate].width / 2;
    y_m = bx->Boxes[candidate].y + bx->Boxes[candidate].height / 2;
    bxok = true;
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
}

void Depth2PointCloud::processDepth(const sensor_msgs::ImageConstPtr& _img)
{
    pthread_mutex_lock( &this->count_mutex );
    cv_bridge::CvImagePtr in_imgPtr = cv_bridge::toCvCopy(_img, sensor_msgs::image_encodings::TYPE_32FC1);
    mask = Mat::zeros(in_imgPtr->image.size(), CV_8UC1);

    if(pcok && bxok)
    {
        Vec3f point3D;
        geometry_msgs::PointStamped point;
        point3D[0] = 0; point3D[1] = 0; point3D[2] = 0;
        for(uint j = xmin; j < xmin+width; j++)
        {
            for(uint i = ymin; i < ymin+height; i++)
            {
                point3D = getDepth(in_imgPtr->image, j, i , cx, cy, fx, fy);
                if(j == x_m && i == y_m)
                {
                    point.point.x = point3D[0];
                    point.point.y = point3D[1];
                    point.point.z = point3D[2];
                }
                if(point3D[2] < dd)
                    mask.at<uint8_t>(i, j) = 1;
        // pc->points.push_back( pcl::PointXYZ( point3D[0], point3D[1], point3D[2] ) );
            }
        }
        // rectangle(xmin, ymin, width, height);
        mkok = true;
        point.header.frame_id = _img->header.frame_id;
        point.header.stamp = ros::Time(0);

        pub_pt.publish(point);
    }
    pthread_mutex_unlock( &this->count_mutex );
}

void Depth2PointCloud::Imageprocessing(const sensor_msgs::ImageConstPtr& _img)
{
    // pthread_mutex_lock( &this->count_mutex );
    cv_bridge::CvImagePtr in_ptr;

    in_ptr = cv_bridge::toCvCopy(_img, "bgr8");
    if(pcok && bxok && mkok)
    {
        Mat dst;

        in_ptr->image.copyTo(dst, mask);
        dst = dst(Rect(xmin, ymin, width, height));
        imshow("segment", dst);
        waitKey(1);

         std_msgs::Header header = _img->header;
         header.stamp = ros::Time(0);

         sensor_msgs::ImagePtr img = cv_bridge::CvImage(header, "bgr8", dst).toImageMsg();

         pub_image.publish( img );
    }
    // pthread_mutex_unlock( &this->count_mutex );
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "depth2pointcloud");

	ros::NodeHandle nh;
	Depth2PointCloud depth2pointcloud(nh, 2);

	ros::spin();

	return 0;
}
