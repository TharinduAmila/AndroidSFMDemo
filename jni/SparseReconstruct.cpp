#include <headers/MSparse.h>

using namespace std;
using namespace cv;

int doSparseReconstruction(std::vector<MFramePair>& pairList,
		cv::Mat_<double> K, MGraph& outGraph) {
	vector<MGraph> graphList;
	//find features in images using fast feature detector match using hybrid tracker
	FastFeatureDetector ffd(8, true);
	ffd.detect(pairList[0].img1, pairList[0].imgpts1);
	for (int i = 0; i < pairList.size(); i++) {
		ffd.detect(pairList[i].img2, pairList[i].imgpts2);
		OFmatch(pairList[i]);
		if (i != (pairList.size() - 1)) {
			pairList[i + 1].imgpts1 = pairList[i].imgpts2;
		}
	}
	//find essential matrices for each consecutive image pairs
	for (int i = 0; i < pairList.size(); i++) {
		string print = "Size Before "
				+ convertIntToString(pairList[i].matchPts1.size()) + "\n";
		findEssentialMatrix(pairList[i], K);
		if(pairList[i].matchPts1.size()<minMatchNu){
			return 0;
		}
		print = "Size After " + convertIntToString(pairList[i].matchPts1.size())
				+ "\n";
		__android_log_write(ANDROID_LOG_ERROR, SPARSE_TAG, print.c_str());
		cout << endl;
	}
	for (int i = 0; i < pairList.size(); i++) {
		Matx34d Pose;
		MGraph graph;
		vector<Point3d> pointCloud;
		vector<KeyPoint> k1, k2;
		vector<Point2d> p1, p2;
		PointsToKeyPoints(pairList[i].matchPts1, k1);
		PointsToKeyPoints(pairList[i].matchPts2, k2);
		findCorrectPose(pairList[i].E, k1, k2, K, K.inv(), Pose, pointCloud);
		KeyPointsToPoints(pairList[i].imgpts1, p1);
		KeyPointsToPoints(pairList[i].imgpts2, p2);
		graph.cameraMat = K;
		graph.framesIncluded.push_back(i);
		graph.framesIncluded.push_back(i + 1);
		graph.pointCloud = pointCloud;
		graph.obsValues.push_back(p1);
		graph.obsValues.push_back(p2);
		for (int j = 0; j < graph.pointCloud.size(); j++) {
			vector<int> t;
			t.push_back(pairList[i].matchedIndex1[j]);
			t.push_back(pairList[i].matchedIndex2[j]);
			graph.obsIndexes.push_back(t);
		}
		Matx34d P(1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0);
		graph.relativePoses.push_back(P);
		graph.relativePoses.push_back(Pose);
		graphList.push_back(graph);
	}
	for (int i = 0; i < graphList.size(); i++) {
		std::ostringstream ss;
		ss << graphList[i].relativePoses[0];
		ss << "\n \n";
		ss << graphList[i].relativePoses[1];
		__android_log_write(ANDROID_LOG_DEBUG, "K Matrix", ss.str().c_str());
	}
	//two view bundle adjustment
	for (int i = 0; i < graphList.size(); i++) {
		vector<Matx34d> outPutPose;
		vector<vector<Point2d> > usedpoints;
		vector<Point2d> p1, p2;
		for (int j = 0; j < graphList[i].pointCloud.size(); j++) {
			p1.push_back(
					graphList[i].obsValues[0][graphList[i].obsIndexes[j][0]]);
			p2.push_back(
					graphList[i].obsValues[1][graphList[i].obsIndexes[j][1]]);
		}
		usedpoints.push_back(p1);
		usedpoints.push_back(p2);
		string print = "Going For Bundling \n"+convertIntToString(graphList[i].pointCloud.size())+
				" "+convertIntToString(usedpoints.size());
		__android_log_write(ANDROID_LOG_ERROR, SPARSE_TAG, print.c_str());
		twoViewBundleAdjustment(K, graphList[i].pointCloud, usedpoints,
				graphList[i].relativePoses[1], outPutPose);
		print = "Done Bundling \n";
		__android_log_write(ANDROID_LOG_ERROR, SPARSE_TAG, print.c_str());
		//re triangulation
		vector<KeyPoint> n1, n2;
		PointsToKeyPoints(p1, n1);
		PointsToKeyPoints(p2, n2);
		vector<Point3d> newPoints;
		double error = TriangulatePoints(n1, n2, Mat(K), Mat(K.inv()),
				outPutPose[0], outPutPose[1], newPoints);
		graphList[i].relativePoses = outPutPose;
		graphList[i].pointCloud = newPoints;
	}
	//merge point clouds in different 3d frames into one
	outGraph = mergePointClouds(graphList);
	//removeOutliers(outGraph);
	string print = "All done number of points"
			+ convertIntToString(outGraph.pointMap.size()) + " cloud "
			+ convertIntToString(outGraph.pointCloud.size()) + "\n";
	__android_log_write(ANDROID_LOG_ERROR, SPARSE_TAG, print.c_str());
	return 1;
}
