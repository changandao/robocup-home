#include <darknet_ros_msgs/BoundingBoxes.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <all_msgs/Boxes.h>
#include <all_msgs/Box.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include <tf/transform_listener.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace cv;

//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;

class Yolo_roi
{

    private:
      //! The node handle
      ros::NodeHandle nh_;
      //! Node handle in the private namespace
      ros::NodeHandle priv_nh_;

      image_transport::ImageTransport it;

      //! Define publishers and subscribers
      ros::Publisher pub_box;
      // ros::ServiceClient client_box;

      ros::Subscriber sub_camInfo, sub_bx;
      image_transport::Subscriber sub_img;
      // ros::Subscriber sub2_;

      bool pcok;
      bool isnan_(float fnan);

      Mat image;

      float fx, fy, cx, cy;

      void processBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& bx);
      // void filterBoxes(const sensor_msgs::ImageConstPtr& img);

      Vec3f getDepth(const Mat& depthImage, int x, int y, float cx, float cy, float fx, float fy);
      void CameraInfo(const sensor_msgs::CameraInfoConstPtr& info);

      pthread_mutex_t count_mutex;
      
    public:
      //! Subscribes to and advertises topics
      Yolo_roi(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"), it(nh)
      {
        count_mutex = PTHREAD_MUTEX_INITIALIZER;
        pcok = false;

        sub_camInfo = nh_.subscribe("/xtion/rgb/camera_info", 10, &Yolo_roi::CameraInfo, this);
        sub_bx = nh_.subscribe("/darknet_ros/bounding_boxes", 10, &Yolo_roi::processBoxes, this);
        // sub_img = it.subscribe("/xtion/depth_registered/image_raw", 10, &Yolo_roi::filterBoxes, this);

        pub_box = nh_.advertise<all_msgs::Boxes>("/God_watcher/Boxes", 10);

        ROS_INFO("yolo_roi initialized ...");
      }

      ~Yolo_roi() {}
};

void Yolo_roi::CameraInfo(const sensor_msgs::CameraInfoConstPtr& _info)
{
    fx = _info->K.at(0);
    fy = _info->K.at(4);
    cx = _info->K.at(2);
    cy = _info->K.at(5);
    pcok = true;
}

bool Yolo_roi::isnan_(float fnan)
{
  return(!fnan==fnan);
}

cv::Vec3f Yolo_roi::getDepth(const cv::Mat & depthImage, int x, int y, float cx, float cy, float fx, float fy)
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

// void Yolo_roi::filterBoxes(const sensor_msgs::ImageConstPtr &img)
// {
//   pthread_mutex_lock( &this->count_mutex );
//     cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
//     image = imgPtr->image.clone();
//    pthread_mutex_unlock( &this->count_mutex );
// }

void Yolo_roi::processBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& bx)
{
<<<<<<< HEAD
  pthread_mutex_lock( &this->count_mutex );
=======
    float prob = 0.7;
    all_msgs::Boxes Boxes_to_deal;
    Boxes_to_deal.header = bx->header;
    cv_bridge::CvImagePtr in_imgPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
    if(pcok)
    {
        float prob = 0.75;
        all_msgs::Boxes Boxes_to_deal;
        Boxes_to_deal.header = bx->header;
        Vec3f point3D;
        for(int i = 0; i < bx->bounding_boxes.size(); i++)
        {
<<<<<<< HEAD
          if(bx->bounding_boxes[i].probability > prob)
=======
          all_msgs::Box bb;
          bb.x = bx->bounding_boxes[i].xmin;
          bb.y = bx->bounding_boxes[i].ymin;
          bb.height = bx->bounding_boxes[i].ymax - bx->bounding_boxes[i].ymin;
          bb.width = bx->bounding_boxes[i].xmax - bx->bounding_boxes[i].xmin;
          bb.probability = bx->bounding_boxes[i].probability;

          int x_m = bb.x + bb.width / 2;
          int y_m = bb.y + bb.height / 2;
          point3D = getDepth(in_imgPtr->image, x_m, y_m, cx, cy, fx, fy);
          if( !(isnan_(point3D[0]) | isnan_(point3D[1]) | isnan_(point3D[2])) && point3D[2] < 3)
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
          {
            all_msgs::Box bb;
            bb.x = bx->bounding_boxes[i].xmin;
            bb.y = bx->bounding_boxes[i].ymin;
            bb.height = bx->bounding_boxes[i].ymax - bx->bounding_boxes[i].ymin;
            bb.width = bx->bounding_boxes[i].xmax - bx->bounding_boxes[i].xmin;
            bb.probability = bx->bounding_boxes[i].probability;

            // int x_m = bb.x + bb.width / 2;
            // int y_m = bb.y + bb.height / 2;
            // point3D = getDepth(image, x_m, y_m, cx, cy, fx, fy);
            // if( !(isnan_(point3D[0]) | isnan_(point3D[1]) | isnan_(point3D[2])) && point3D[2] < 5)
            // {
              bb.header = bx->header;
              bb.id = "person";
              Boxes_to_deal.Boxes.push_back(bb);
            // }
          }
        }
        pub_box.publish(Boxes_to_deal);
    }
    pthread_mutex_unlock( &this->count_mutex );
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolo_roi");
    ros::NodeHandle nh;

    Yolo_roi yolo_roi(nh);

    ros::spin();
    return 0;
}

