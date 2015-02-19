#ifndef SPARSE_H
#define SPARSE_H
#include <opencv2/opencv.hpp>
#include <headers/MFrame.h>
#include <headers/Mtracking.h>
#include <headers/MGraph.h>
#include <headers/Mutils.h>
#include <headers/Mtriangulation.h>
#include <headers/MbundleAdjustment.h>

#include <android/log.h>
#define SPARSE_TAG "SparseReconstructor"
const int minMatchNu = 40;
int doSparseReconstruction(std::vector<MFramePair>& pairList, cv::Mat_<double> K,MGraph& outGraph);//
#endif
