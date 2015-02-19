#ifndef UTILS_H
#define UTILS_H
#include <opencv2/opencv.hpp>
#include <headers/MFrame.h>
#include <headers/Mtriangulation.h>
#include <headers/MbundleAdjustment.h>
#include <android/log.h>

void createPoseFromRotationTranslation(cv::Mat_<double> R, cv::Mat_<double> t,
		cv::Matx34d& Pose);
void KeyPointsToPoints(std::vector<cv::KeyPoint> in,
		std::vector<cv::Point2f>& out);
void KeyPointsToPoints(std::vector<cv::KeyPoint> in,
		std::vector<cv::Point2d>& out);
void PointsToKeyPoints(std::vector<cv::Point2f> in,
		std::vector<cv::KeyPoint>& out);
void PointsToKeyPoints(std::vector<cv::Point2d> in,
		std::vector<cv::KeyPoint>& out);
void Point2fToPoint2d(std::vector<cv::Point2f> in,
		std::vector<cv::Point2d>& out);
void putTextOnTopOfMat(cv::Mat & in, std::string print, double scale,
		int thickness);
std::string convertIntToString(int in);
void findEssentialMatrix(MFramePair& pair, cv::Mat_<double> K);
bool CheckCoherentRotation(cv::Mat_<double>& R);
double getPointCloudAndPointsInFront(cv::Matx34d P, cv::Matx34d Pose,
		std::vector<cv::KeyPoint> key1, std::vector<cv::KeyPoint> key2,
		cv::Mat_<double> K, cv::Mat_<double> Kinv,
		std::vector<cv::Point3d>& pointCloud);
bool findCorrectPose(cv::Mat_<double> E, std::vector<cv::KeyPoint> key1,
		std::vector<cv::KeyPoint> key2, cv::Mat_<double> K, cv::Mat_<double> Kinv, cv::Matx34d& out,
		std::vector<cv::Point3d>& cloudOut);
MGraph mergePointClouds(std::vector<MGraph> graphList);
void extractRfromePose(cv::Matx34d relativePose,cv::Mat_<double>& out);
void extractTfromePose(cv::Matx34d relativePose,cv::Mat_<double>& out);
double getReprojectionError(cv::Mat_<double> K,cv::Mat_<double> P1,cv::Mat_<double> X,cv::Point2f kp1);
void removeOutliers(MGraph& in);
#endif
