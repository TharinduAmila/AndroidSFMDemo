#ifndef TRIANGULATION_H
#define TRIANGULATION_H
#define EPSILON 0.000001
#include <opencv2/opencv.hpp>
#include <headers/MGraph.h>
#include <android/log.h>

double TriangulatePoints(const std::vector<cv::KeyPoint>& pt_set1,
						 const std::vector<cv::KeyPoint>& pt_set2, const cv::Mat& K, const cv::Mat&Kinv,
						 const cv::Matx34d& P, const cv::Matx34d& P1, std::vector<cv::Point3d>& pointcloud);
void TriangulateGraphPoints(MGraph& graph);
#endif
