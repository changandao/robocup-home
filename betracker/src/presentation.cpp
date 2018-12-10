#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/videoio.hpp>
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
    
    VideoWriter writer;


//    void BigCallback(const sensor_msgs::ImageConstPtr& img1, const sensor_msgs::ImageConstPtr& img2, const all_msgs::BoxesConstPtr& bx);
    void BoxesCallback(const all_msgs::BoxesConstPtr& bx);
    void ImgrectCallback(const sensor_msgs::ImageConstPtr& _img_rect);
    // void segcallback(const sensor_msgs::ImageConstPtr& img);

//        void CameraInfo(const sensor_msgs::CameraInfoConstPtr& _info);


public:
    DepthProcessing(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"), it(nh)
    {
        pcok = false;
        imgok1 = false;
        imgok2 = false;
	    writer = VideoWriter("/home/atHomeSS18/Test1.avi", CV_FOURCC('M', 'J', 'P', 'G'), 15.0, Size(640, 480), true);
        sub_boxes = nh_.subscribe("/God_watcher/Boxes", 10, &DepthProcessing::BoxesCallback, this);
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

    return rois;
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
                rectangle(dst, Rois[i].tl(), Rois[i].br(), 2, 8, 0);
                if(labels.at<float>(i,0) > 0)
                {
                    putText(dst, "positive", Rois[i].tl(), font, 3, (0,255,255), 2);
                }
                else
                {
                    putText(dst, "negative", Rois[i].tl(), font, 3, (0,255,255), 2);
                }
            }
        }
        imshow("segmented", dst);
        waitKey(1);
        
        this->writer << dst;

        // std_msgs::Header header = _img->header;
        // header.stamp = ros::Time(0);

        // sensor_msgs::ImagePtr img = cv_bridge::CvImage(header, "bgr8", dst).toImageMsg();

        // pub_image.publish( img );

    }
    // pthread_mutex_unlock(&this->count_mutex);
}

void DepthProcessing::BoxesCallback(const all_msgs::BoxesConstPtr& bx)
{
    
    //pthread_mutex_lock( &this->count_mutex );
    Bxes = all_msgs::Boxes(*bx);

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
