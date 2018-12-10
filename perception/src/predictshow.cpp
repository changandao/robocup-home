#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml.hpp>
#include <geometry_msgs/PointStamped.h>

//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

#include <all_msgs/Boxes.h>
#include <all_msgs/Object.h>
#include <all_msgs/ObjectArray.h>
#include <tf/transform_listener.h>

using namespace cv;
using namespace std;
using namespace cv::ml;

class DepthProcessing
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    ros::Subscriber sub_camInfo, sub_boxes;
    ros::Publisher pub_obj;

//    message_filters::Subscriber<sensor_msgs::Image> raw_image_sub;
//    message_filters::Subscriber<sensor_msgs::Image> rect_image_sub;
//    message_filters::Subscriber<all_msgs::Boxes> boxes_sub;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub_img_raw;
    image_transport::Subscriber sub_img_rect, sub_seg;

    tf::TransformListener listener_;

    Ptr<SVM> svm;

    bool pcok, imgok1, imgok2;
    float fx, fy, cx, cy;
    // Mat img_raw, img_rect;
    Mat mask;
    all_msgs::Boxes Bxes;

    pthread_mutex_t count_mutex;



    Vec3f getDepth(const Mat& depthImage, int x, int y, float cx, float cy, float fx, float fy);

    void depthCameraInfo(const sensor_msgs::CameraInfoConstPtr& _info);
//    void BigCallback(const sensor_msgs::ImageConstPtr& img1, const sensor_msgs::ImageConstPtr& img2, const all_msgs::BoxesConstPtr& bx);
    void BoxesCallback(const all_msgs::BoxesConstPtr& bx);
    void ImgrawCallback(const sensor_msgs::ImageConstPtr& _img_raw);
    void ImgrectCallback(const sensor_msgs::ImageConstPtr& _img_rect);
    // void segcallback(const sensor_msgs::ImageConstPtr& img);

    vector<Rect> box_regression(Mat& img);
    vector<float> getCM(Mat& img);
    float getSkewness(Mat& img, Scalar& mean, Scalar& std);
    Mat predict(vector<Rect>& rects, Mat& img);

//        void CameraInfo(const sensor_msgs::CameraInfoConstPtr& _info);


public:
    DepthProcessing(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"), it(nh)
    {
        pcok = false;
        imgok1 = false;
        imgok2 = false;

//                message_filters::Subscriber<sensor_msgs::Image> raw_image_sub(nh, "/xtion/depth_registered/image_raw", 1);
//                message_filters::Subscriber<sensor_msgs::Image> rect_image_sub(nh, "/xtion/rgb/image_rect_color", 1);
//                message_filters::Subscriber<all_msgs::Boxes> boxes_sub(nh, "/God_watcher/Boxes", 1);

        sub_camInfo = nh_.subscribe("/xtion/rgb/camera_info", 10, &DepthProcessing::depthCameraInfo, this);
        // sub_seg = it.subscribe("/depth2pointcloud/Segmented_Image", 10, &DepthProcessing::segcallback, this);
        sub_boxes = nh_.subscribe("/God_watcher/Boxes", 10, &DepthProcessing::BoxesCallback, this);
        sub_img_raw = it.subscribe("/xtion/depth_registered/image_raw", 10, &DepthProcessing::ImgrawCallback, this);
        sub_img_rect = it.subscribe("/xtion/rgb/image_rect_color", 10, &DepthProcessing::ImgrectCallback, this);
//                message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, all_msgs::Boxes> sync(raw_image_sub, rect_image_sub, boxes_sub, 10);
//                sync.registerCallback(&DepthProcessing::BigCallback, this);

        // pub_obj = priv_nh_.advertise<all_msgs::ObjectArray>("ObjectArray", 10);

        svm = SVM::load("/home/atHomeSS18/test_workspace/src/perception/src/svm_trained.xml");
        count_mutex = PTHREAD_MUTEX_INITIALIZER;

// mask = Mat::zeros(in_imgPtr->image.size(), CV_8UC1);
        ROS_INFO("DepthProcessing begins....");
    }

    ~DepthProcessing() {};

};

Mat DepthProcessing::predict( vector<Rect>& rects, Mat& img )
{
//    ROS_INFO("IM HERE 1");
    Mat hsv, response, FeatureMat;
    // vector<Mat> Rois;
    // // Mat response = Mat::zeros(Size(1, 1), CV_32SC1);
    // Rois = box_regression(img);
    for(int i = 0; i < rects.size(); i++)
    {
        cvtColor(img(rects[i]), hsv, CV_BGR2HSV);
        Mat feature(getCM(hsv), true);
        feature = feature.reshape(0,1);
        FeatureMat.push_back(feature);
    }

//     cvtColor(temp, hsv, CV_BGR2HSV);
//     Mat FeatureMat(getCM(hsv), true);
// //    ROS_INFO("IM HERE 7");
//     FeatureMat = FeatureMat.reshape(0, 1);
//     ROS_INFO("IM HERE 8");
//     cout << FeatureMat.type() << endl;
//     cout << FeatureMat << endl;
//     cout << response.type() << endl;
    // cout << FeatureMat.type() << endl;
    svm->predict(FeatureMat, response);
    // ROS_INFO("IM HERE 9");

    return response;
}

vector<Rect> DepthProcessing::box_regression(Mat& img)
{
//    ROS_INFO("IM HERE 2");
    Mat gray;
    cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    vector<std::vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;

    findContours(gray, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
    vector<vector<Point> > contours_poly( contours.size() );
    // vector<Rect> boundRect( contours.size() );
    vector<Rect> rois;
    Rect rect;

    for ( uint i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( Mat(contours[i]), contours_poly[i], 20, true );
          rect = cv::boundingRect( cv::Mat(contours_poly[i]) );
          if(rect.width > 70 && rect.height > 70)
          {
            // boundRect.push_back(rect);
            rois.push_back(rect);

          }
    }

    // Rect box;
    // int size = 0;

    // for ( uint i = 0; i < boundRect.size(); i++ )
    // {
    //     if (boundRect[i].height * boundRect[i].width > size)
    //     {
    //         size = boundRect[i].height * boundRect[i].width;
    //         box = boundRect[i];
    //     }
    // }

    // Mat dst;
    // dst = img(box);

    return rois;
}

vector<float> DepthProcessing::getCM(Mat& img)
{
    // ROS_INFO("IM HERE 3");
    vector<Mat> HSV(3);
    vector<float> CM;
    split(img, HSV);
    Scalar h_mean, h_std, s_mean, s_std, v_mean, v_std;
    meanStdDev(HSV[0], h_mean, h_std);
    meanStdDev(HSV[1], s_mean, s_std);
    meanStdDev(HSV[2], v_mean, v_std);
    CM.push_back((float)h_mean.val[0]);
    CM.push_back((float)s_mean.val[0]);
    CM.push_back((float)v_mean.val[0]);
    CM.push_back((float)h_std.val[0]);
    CM.push_back((float)s_std.val[0]);
    CM.push_back((float)v_std.val[0]);

    float h_skew, s_skew, v_skew;
    h_skew = getSkewness(HSV[0], h_mean, h_std);
    s_skew = getSkewness(HSV[1], s_mean, s_std);
    v_skew = getSkewness(HSV[2], v_mean, v_std);

    CM.push_back(h_skew);
    CM.push_back(s_skew);
    CM.push_back(v_skew);

    return CM;

}

float DepthProcessing::getSkewness(Mat& img, Scalar& mean, Scalar& std)
{
    // ROS_INFO("IM HERE 4");
    float skew;
    float sum = 0.0;
    for (uint i = 0; i < img.rows; i++)
    {
        for (uint j = 0; j < img.cols; j++)
        {
            sum += pow(img.at<uint8_t>(i, j) - mean.val[0], 3);
        }
    }
    if (std.val[0] != 0 && img.rows * img.cols != 0 )
        skew = sum / (img.rows * img.cols) / pow(std.val[0], 3);
    else
        return 0;

    return skew;
}

void DepthProcessing::depthCameraInfo(const sensor_msgs::CameraInfoConstPtr& _info)
{
    fx = _info->K.at(0);
    fy = _info->K.at(4);
    cx = _info->K.at(2);
    cy = _info->K.at(5);
    pcok = true;
}

cv::Vec3f DepthProcessing::getDepth(const cv::Mat & depthImage, int x, int y, float cx, float cy, float fx, float fy)
{
    if (!(x >= 0 && x < depthImage.cols && y >= 0 && y < depthImage.rows))
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
    float unit_scaling = isInMM ? 0.001f : 1.0f;
    float constant_x = unit_scaling / fx; //cameraInfo.K.at(0)
    float constant_y = unit_scaling / fy; //cameraInfo.K.at(4)
    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    float depth;
    bool isValid;
    if (isInMM)
    {
        depth = (float)depthImage.at<uint16_t>(y, x);
        isValid = depth != 0.0f;
    }
    else
    {
        depth = depthImage.at<float>(y, x);
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
        pt[2] = depth * unit_scaling;

    }
    return pt;
}

void DepthProcessing::ImgrawCallback(const sensor_msgs::ImageConstPtr& _img_raw)
{
    
    // all_msgs::Box b;
    // std::vector<all_msgs::Box> boxes;
    // //pthread_mutex_lock( &this->count_mutex );
    // // Copy boxes to aux variable    
    


    //pthread_mutex_unlock( &this->count_mutex );

    cv_bridge::CvImagePtr in_imgPtr = cv_bridge::toCvCopy(_img_raw, sensor_msgs::image_encodings::TYPE_32FC1);
    mask = Mat::zeros(in_imgPtr->image.size(), CV_8UC1);

    if(pcok && imgok1)
    {
        Vec3f point3D;
        point3D[0] = 0; point3D[1] = 0; point3D[2] = 0;
        // Mat dst;
        // // Mat mask = Mat::zeros(img_raw.size(), CV_8UC1);
        for(int i=0; i < Bxes.Boxes.size(); i++)
        {
            int xmin = Bxes.Boxes[i].x;
            int ymin = Bxes.Boxes[i].y;
            int xmax = xmin + Bxes.Boxes[i].width;
            int ymax = ymin + Bxes.Boxes[i].height;

            for(uint j = xmin; j < xmax; j++)
            {
                for(uint k = ymin; k < ymax; k++)
                {
                    point3D = getDepth(in_imgPtr->image, j, k , cx, cy, fx, fy);
                    if(point3D[2] < 3)
                        mask.at<uint8_t>(k, j) = 1;
                }
            }
        }
    }
    imgok2 = true;
    
}

void DepthProcessing::ImgrectCallback(const sensor_msgs::ImageConstPtr& _img_rect)
{
    // pthread_mutex_lock(&this->count_mutex);
    cv_bridge::CvImagePtr in_ptr;

    in_ptr = cv_bridge::toCvCopy(_img_rect, "bgr8");
    if(pcok && imgok2 && imgok1)
    {
        Mat dst, labels;

        in_ptr->image.copyTo(dst, mask);
        Mat process = dst.clone();
        vector<Rect> Rois = box_regression(process);
        if(Rois.size() > 0)
        {    labels = predict(Rois, process);
            int font = FONT_HERSHEY_PLAIN;
            for(int i = 0; i < Rois.size(); i++)
            {
                if(labels.at<float>(i,0) > 0)
                    putText(dst, "positive", Rois[i].tl(), font, 3, (255,255,255), 2);
                else
                    putText(dst, "negative", Rois[i].tl(), font, 3, (255,255,255), 2);
            }
        }
        imshow("segmented", dst);
        waitKey(1);

        // std_msgs::Header header = _img->header;
        // header.stamp = ros::Time(0);

        // sensor_msgs::ImagePtr img = cv_bridge::CvImage(header, "bgr8", dst).toImageMsg();

        // pub_image.publish( img );

    }
    // pthread_mutex_unlock(&this->count_mutex);
}

void DepthProcessing::BoxesCallback(const all_msgs::BoxesConstPtr& bx)
{
    // pthread_mutex_lock(&this->count_mutex);
    // if(pcok && imgok1 && imgok2)
    // {
    //     Vec3f point3D;
    //     point3D[0] = 0; point3D[1] = 0; point3D[2] = 0;
    //     Mat dst;
    //     Mat mask = Mat::zeros(img_raw.size(), CV_8UC1);
    //     for(int i=0; i < bx->Boxes.size(); i++)
    //     {
    //         int xmin = bx->Boxes[i].x;
    //         int ymin = bx->Boxes[i].y;
    //         int xmax = xmin + bx->Boxes[i].width;
    //         int ymax = ymin + bx->Boxes[i].height;

    //         for(uint j = xmin; j < xmax; j++)
    //         {
    //             for(uint k = ymin; k < ymax; k++)
    //             {
    //                 point3D = getDepth(img_raw, j, k , cx, cy, fx, fy);
    //                 if(point3D[2] < 3)
    //                     mask.at<uint8_t>(j, k) = 1;
    //             }
    //         }
    //     }
    //     img_rect.copyTo(dst, mask);
    //     imshow("segmented", dst);
    //     waitKey(1);
    // }
    // pthread_mutex_unlock(&this->count_mutex);

    //pthread_mutex_lock( &this->count_mutex );
    Bxes = all_msgs::Boxes(*bx);

    /*all_msgs::Box b;
    ROS_INFO("n bbox: %d", (int)Bxes.Boxes.size());
    for (int i=0; i < Bxes.Boxes.size(); ++i) {
        //ROS_INFO("Adding box");
        //ROS_INFO("number: %d", (int)Bxes.Boxes.size());
        b.id = Bxes.Boxes[i].id;
        b.probability  =Bxes.Boxes[i].probability;
        b.x  =Bxes.Boxes[i].x;
        b.y  =Bxes.Boxes[i].y;
        b.height  =Bxes.Boxes[i].height;
        b.width=  Bxes.Boxes[i].width;

        //boxes.Boxes.push_back(b);
    }*/

    //pthread_mutex_unlock( &this->count_mutex );
   

    imgok1 = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthprocessing");
    ros::NodeHandle nh;

    DepthProcessing depthprocessing(nh);
    // ros::AsyncSpinner spinner(4);
    // spinner.start();

    ros::spin();
    // ros::waitForShutdown();
    return 0;
}
