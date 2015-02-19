#ifndef DATA_H
#define DATA_H

#include <vector>
#include <opencv2\core\core.hpp>
#include <opencv2\features2d\features2d.hpp>

class worldData {
public:
	cv::Point3d point;
	std::vector<int> track;
};
class MGraph {
public:
	cv::Mat_<double> cameraMat;
	std::map<int, worldData> pointMap;
	std::vector<int> framesIncluded;
	std::vector<std::vector<cv::Point2d> > obsValues;
	std::vector<std::vector<int> > obsIndexes;
	std::vector<cv::Point3d> pointCloud;
	std::vector<cv::Matx34d> relativePoses;
};
#endif
