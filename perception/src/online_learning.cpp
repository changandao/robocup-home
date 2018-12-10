#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml.hpp>
#include <all_msgs/send_flags.h>
#include <std_srvs/Empty.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tum_alle_athome_speech_msgs/srvTTS.h>

#include <boost/filesystem.hpp>
#include <math.h>

using namespace cv;
using namespace cv::ml;
using namespace std;

typedef boost::filesystem::directory_iterator direct_it;
<<<<<<< HEAD
string negativePath = "/home/atHomeSS18/training_data/negative";
string positivePath = "/home/atHomeSS18/training_data/operator";
=======
string negativePath = "/home/atHomeSS18/God-Watcher/workspace/Finalproject/src/train_data/negative/";
string positivePath = "/home/atHomeSS18/God-Watcher/workspace/Finalproject/src/train_data/operator/";
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
// string savePath = "";
bool start = false;

class Online_learning
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

	Ptr<SVM> svm;
	Ptr<TrainData> trainData;
	Mat box_regression(Mat& img);
	vector<float> getCM(Mat& img);
	float getSkewness(Mat& img, Scalar& mean, Scalar& std);
	// string positivePath, negativePath;
	vector<Mat> get_train_data(string pPath, string nPath);
	vector<Mat> Train_Data;
	// bool pred_start;

	ros::ServiceServer learn_begin, predict_begin;
	ros::ServiceClient learn_finish, client_speech_;

	// image_transport::ImageTransport it_;
	// image_transport::Subscriber sub_img;

<<<<<<< HEAD
        bool Callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
=======
	bool Callback(all_msgs::send_flags::Request &req, all_msgs::send_flags::Response &res);
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e


public:
	Online_learning(ros::NodeHandle nh) : nh_(nh), priv_nh_(nh)//, it_(nh)
	{
		//learn_finish = nh_.serviceClient<all_msgs::send_flags>("/online_learning/finish");
		// svm = SVM::create();
		// svm->setType(SVM::C_SVC);
		// svm->setKernel(SVM::RBF);
		// // negativePath = "/home/tiago/small_god/src/train_data/negative/";
		// // positivePath = "/home/tiago/small_god/src/train_data/operator/";
		//   	Train_Data = get_train_data(positivePath, negativePath);
		//   	trainData = TrainData::create(Train_Data[0], ROW_SAMPLE, Train_Data[1]);
		//   	svm->trainAuto(trainData, 5, SVM::getDefaultGrid(SVM::C), SVM::getDefaultGrid(SVM::GAMMA), SVM::getDefaultGrid(SVM::P), SVM::getDefaultGrid(SVM::NU), SVM::getDefaultGrid(SVM::COEF), SVM::getDefaultGrid(SVM::DEGREE), true);

		//   	svm->save("svm_trained.xml");
		// all_msgs::send_flags Flag;
		// Flag.request.flag = 1;
		//learn_finish.call(Flag);
		svm = SVM::create();
		predict_begin = nh_.advertiseService("/online_learning/start", &Online_learning::Callback, this);

		// sub_img = it_.subscribe("/depth2pointcloud/Segmented_Image", 1, &Online_learning::PredictCallback, this);
		// client_speech_ = nh_.serviceClient<tum_alle_athome_speech_msgs::srvTTS>("/tiago/speech/tts");

		// pred_start = false;

		ROS_INFO("online_learning begins...");
	}

	~Online_learning()
	{
		ROS_INFO("online_learning no more needed..");
	}

};

<<<<<<< HEAD
bool Online_learning::Callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
=======
bool Online_learning::Callback(all_msgs::send_flags::Request &req, all_msgs::send_flags::Response &res)
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
{
	svm->setType(SVM::C_SVC);
	svm->setKernel(SVM::RBF);
	// negativePath = "/home/tiago/small_god/src/train_data/negative/";
	// positivePath = "/home/tiago/small_god/src/train_data/operator/";
	Train_Data = get_train_data(positivePath, negativePath);
	trainData = TrainData::create(Train_Data[0], ROW_SAMPLE, Train_Data[1]);
	svm->trainAuto(trainData, 5, SVM::getDefaultGrid(SVM::C), SVM::getDefaultGrid(SVM::GAMMA), SVM::getDefaultGrid(SVM::P), SVM::getDefaultGrid(SVM::NU), SVM::getDefaultGrid(SVM::COEF), SVM::getDefaultGrid(SVM::DEGREE), true);
<<<<<<< HEAD
        ROS_INFO("Learning complete!");
        svm->save("/home/atHomeSS18/God-Watcher/workspace/Finalproject/src/perception/src/svm_trained.xml");
        trainData.release();
=======

        svm->save("/home/atHomeSS18/God-Watcher/workspace/Finalproject/src/perception/src/svm_trained.xml");
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
	return true;
}

// void Online_learning::speak(string text)
// {
//     ROS_INFO_STREAM(text);
//     tum_alle_athome_speech_msgs::srvTTS msg_speech;
//     msg_speech.request.text = text;
//     client_speech_.call(msg_speech);
//     sleep((unsigned int)(text.size()/150.0));
//     ROS_INFO_STREAM("Done talking.");
// }

// void Online_learning::PredictCallback(const sensor_msgs::ImageConstPtr& _img)
// {
// 	if(start && pred_start)
// 	{
//         cv_bridge::CvImagePtr cv_ptr;
//         cv_ptr = cv_bridge::toCvCopy(_img, sensor_msgs::image_encodings::BGR8);
//         float response = predict(cv_ptr->image);
//         if((int) response == 1)
//         	speak("Hello, master!");
//         else if(  (int) response == -1)
//         	speak("You are not my master!");
//         pred_start = false;
// 	}
// }

// float Online_learning::predict( Mat& img )
// {
//     Mat response, hsv;
//     cvtColor(box_regression(img), hsv, CV_BGR2HSV);
//     Mat FeatureMat(getCM(hsv), true);
//     FeatureMat = FeatureMat.reshape(0,1);
//     svm->predict(FeatureMat, response);

//     return response.at<float>(0,0);
// }



Mat Online_learning::box_regression(Mat& img)
{
	Mat gray;
	cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	vector<std::vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	findContours(gray, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );

	for ( uint i = 0; i < contours.size(); i++ )
	{
		approxPolyDP( Mat(contours[i]), contours_poly[i], 20, true );
		boundRect[i] = boundingRect( Mat(contours_poly[i]) );
	}

	Rect box;
	int size = 0;

	for ( uint i = 0; i < boundRect.size(); i++ )
	{
		if (boundRect[i].height * boundRect[i].width > size)
		{
			size = boundRect[i].height * boundRect[i].width;
			box = boundRect[i];
		}
	}

	Mat dst;
	dst = img(box);

	return dst;
}

vector<float> Online_learning::getCM(Mat& img)
{
	vector<Mat> HSV(3);
	vector<float> CM;
	split(img, HSV);
	Scalar h_mean, h_std, s_mean, s_std;
	meanStdDev(HSV[0], h_mean, h_std);
	meanStdDev(HSV[1], s_mean, s_std);
//	meanStdDev(HSV[2], v_mean, v_std);
	CM.push_back(h_mean.val[0]);
	CM.push_back(s_mean.val[0]);
//	CM.push_back(v_mean.val[0]);
	CM.push_back(h_std.val[0]);
	CM.push_back(s_std.val[0]);
//	CM.push_back(v_std.val[0]);

	float h_skew, s_skew;
	h_skew = getSkewness(HSV[0], h_mean, h_std);
	s_skew = getSkewness(HSV[1], s_mean, s_std);
//	v_skew = getSkewness(HSV[2], v_mean, v_std);

	CM.push_back(h_skew);
	CM.push_back(s_skew);
//	CM.push_back(v_skew);

	return CM;

}

float Online_learning::getSkewness(Mat& img, Scalar& mean, Scalar& std)
{
	float skew;
	float sum = 0.0;
	for (uint i = 0; i < img.rows; i++)
	{
		for (uint j = 0; j < img.cols; j++)
		{
			sum += pow(img.at<uint8_t>(i, j) - mean.val[0], 3);
		}
	}
<<<<<<< HEAD
	if(img.rows*img.cols > 0 && std.val[0] != 0)
		skew = sum / (img.rows * img.cols) / pow(std.val[0], 3);
	else
		skew = 0;
=======
	skew = sum / (img.rows * img.cols) / pow(std.val[0], 3);
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e

	return skew;
}

vector<Mat> Online_learning::get_train_data(string pPath, string nPath)
{
	vector<Mat> TRAIN;
	Mat FeatureMat;
	Mat src, hsv;
	direct_it itEnd;
	vector<int> label;

	for (direct_it itor(pPath.c_str()); itor != itEnd; ++itor)
	{
		label.push_back(1);
		string file = itor->path().string();
		boost::filesystem::path filePath(file);

		src = imread(filePath.string(), 1);
		src = box_regression(src);
		cvtColor(src, hsv, CV_BGR2HSV);
		Mat feature(getCM(hsv), true);
		feature = feature.reshape(0, 1);
		FeatureMat.push_back(feature);
	}

	for (direct_it itor(nPath.c_str()); itor != itEnd; ++itor)
	{
		label.push_back(-1);
		string file = itor->path().string();
		boost::filesystem::path filePath(file);

		src = imread(filePath.string(), 1);
		src = box_regression(src);
		cvtColor(src, hsv, CV_BGR2HSV);
		Mat feature(getCM(hsv), true);
		feature = feature.reshape(0, 1);
		FeatureMat.push_back(feature);
	}
	Mat LabelMat(label, true);

	TRAIN.push_back(FeatureMat);
	TRAIN.push_back(LabelMat);

	return TRAIN;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "online_learning");
	ros::NodeHandle nh;
	// ros::Rate r(10);

	// ros::ServiceServer learn_begin = nh.advertiseService("/online_learning/start", Callback);

	// while (ros::ok())
	// {
	// 	if (start)
	// 	{
	// 		break;
	// 	}
	// 	ros::spinOnce();
	// 	r.sleep();
	// }

	Online_learning online_learning(nh);


	ros::spin();

	return 0;
}
