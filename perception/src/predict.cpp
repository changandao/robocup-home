#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml.hpp>

// #include <boost/filesystem.hpp>
#include <math.h>

using namespace cv;
using namespace cv::ml;
using namespace std;

// typedef boost::filesystem::directory_iterator direct_it;
// string predictPath = "/home/tiago/predict/";

class Prediction
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

	image_transport::ImageTransport it;
	image_transport::Subscriber sub_seg;

	ros::ServiceServer pred_ser;

	Ptr<SVM> svm;
	bool pred;

	Mat box_regression(Mat& img);
	vector<float> getCM(Mat& img);
	float getSkewness(Mat& img, Scalar& mean, Scalar& std);
	float predict(Mat& img);

public:
	Prediction(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"), it(nh)
	{
		pred = false;
		pred_ser = nh_.advertiseService("/predicting/start", &Prediction::Callback, this);
		svm = SVM::load("/home/atHomeSS18/test_workspace/src/perception/src/svm_trained.xml");
		if(svm->empty())
		{
			ROS_INFO("load svm detector failed!");
			~Prediction();
		}
		it.subscribe("/depth2pointcloud/Segmented_Image", 10, &Prediction::prediction, this);
	}
	// int predict(Mat& img);
	~Prediction() {};

};

float Prediction::predict( Mat& img )
{
	Mat response, hsv;
	cvtColor(box_regression(img), hsv, CV_BGR2HSV);
	Mat FeatureMat(getCM(hsv), true);
	FeatureMat = FeatureMat.reshape(0,1);
	svm->predict(FeatureMat, response);

	return response.at<float>(0,0);
}

Mat Prediction::box_regression(Mat& img)
{
	Mat gray;
	cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    vector<std::vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;

    findContours(gray, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
    vector<vector<Point> > contours_poly( contours.size() );
  	vector<Rect> boundRect( contours.size() );

  	for( uint i = 0; i < contours.size(); i++ )
  	{
    	approxPolyDP( Mat(contours[i]), contours_poly[i], 20, true );
    	boundRect[i] = boundingRect( Mat(contours_poly[i]) );
  	}

  	Rect box;
    int size = 0;

	for( uint i = 0; i < boundRect.size(); i++ )
	{
		if(boundRect[i].height * boundRect[i].width > size)
		{
	  	size = boundRect[i].height * boundRect[i].width;
	  	box = boundRect[i];
		}
	}
      
    Mat dst;
    dst = img(box);

    return dst;
}

vector<float> Prediction::getCM(Mat& img)
{
	vector<Mat> HSV(3);
	vector<float> CM;
	split(img, HSV);
	Scalar h_mean, h_std, s_mean, s_std, v_mean, v_std;
	meanStdDev(HSV[0], h_mean, h_std);
	meanStdDev(HSV[1], s_mean, s_std);
	// meanStdDev(HSV[2], v_mean, v_std);
	CM.push_back(h_mean.val[0]);
	CM.push_back(s_mean.val[0]);
	// CM.push_back(v_mean.val[0]);
	CM.push_back(h_std.val[0]);
	CM.push_back(s_std.val[0]);
	// CM.push_back(v_std.val[0]);

	float h_skew, s_skew, v_skew;
	h_skew = getSkewness(HSV[0], h_mean, h_std);
	s_skew = getSkewness(HSV[1], s_mean, s_std);
	// v_skew = getSkewness(HSV[2], v_mean, v_std);

	CM.push_back(h_skew);
	CM.push_back(s_skew);
	// CM.push_back(v_skew);

	return CM;	
}

float Prediction::getSkewness(Mat& img, Scalar& mean, Scalar& std)
{
	float skew;
	float sum = 0.0;
	for(uint i = 0; i < img.rows; i++)
	{
		for(uint j = 0; j < img.cols; j++)
		{
			sum += pow(img.at<uint8_t>(i,j) - mean.val[0], 3);
		}
	}
    if (std.val[0] != 0 && img.rows * img.cols != 0 )
        skew = sum / (img.rows * img.cols) / pow(std.val[0], 3);
    else
        return 0;

    return skew;	
}

bool Prediction::Callback()

void Prediction::prediction(sensor_msgs::ImageConstPtr& _img)
{
	if(pred)
	{
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(_img, sensor_msgs::image_encodings::BGR8);
		float label = predict(cv_ptr->image);
		if(label > 0)
		{

		}
		else
		{

		}		
		pred = false;
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "predict");
    ros::NodeHandle nh;	

    Prediction pred(nh);

    ros::spin();

    return 0;
}
