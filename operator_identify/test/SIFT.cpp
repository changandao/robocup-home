#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

int main(int argc, char const *argv[])
{
	cv::Mat src = cv::imread(argv[1], 1);
	cv::Mat gray;
	cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create();
	detector->detectAndCompute(gray, cv::noArray(), keypoints, cv::noArray(), false);

	cv::Mat outimg;
	cv::drawKeypoints(src, keypoints, outimg);

	cv::imshow("Features detected", outimg);
	cv::waitKey(0);

	return 0;
}