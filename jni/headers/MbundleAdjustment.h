#ifndef BUNDLE_H
#define BUNDLE_H
#include <opencv2/opencv.hpp>
#include <android/log.h>
#include <headers/Mutils.h>

void bundleAdjust( std::vector<cv::Point3d>& points, //positions of points in global coordinate system (input and output)
				  const std::vector<std::vector<cv::Point2d> >& imagePoints, //projections of 3d points for every camera
				  const std::vector<std::vector<int> >& visibility, //visibility of 3d points for every camera
				  std::vector<cv::Mat>& cameraMatrix, //intrinsic matrices of all cameras (input and output)
				  std::vector<cv::Mat>& R, //rotation matrices of all cameras (input and output)
				  std::vector<cv::Mat>& T, //translation vector of all cameras (input and output)
				  std::vector<cv::Mat>& distCoeffs, //distortion coefficients of all cameras (input and output)
				  const cv::TermCriteria& criteria,
				  cv::BundleAdjustCallback cb=0, void* user_data=0);
void twoViewBundleAdjustment(cv::Mat_<double> cameraMat, //intrinsic
							 std::vector<cv::Point3d>& points_opt,
							 std::vector<std::vector<cv::Point2d> > matchesUsedFor3d,
							 cv::Matx34d relativePose,std::vector<cv::Matx34d>& out);
void graphBundleAdjustment(MGraph& graph);
#endif
