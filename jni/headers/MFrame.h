#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <opencv2\core\core.hpp>
#include <opencv2\features2d\features2d.hpp>

class MFramePair {
	public:
		cv::Mat img1;
		cv::Mat img2;
		cv::Mat_<double> E;
		cv::Mat F;
		std::vector<cv::KeyPoint> imgpts1;
		std::vector<cv::KeyPoint> imgpts2;
		std::vector<cv::Point2f> matchPts1;
		std::vector<cv::Point2f> matchPts2;
		std::vector<int> matchedIndex1;
		std::vector<int> matchedIndex2;
};
#endif
