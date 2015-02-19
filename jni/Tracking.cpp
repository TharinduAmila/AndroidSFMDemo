#include <headers/Mtracking.h>

using namespace cv;
using namespace std;

void OFmatch(MFramePair& frame) {
	// Detect keypoints in the left and right images
	// making sure images are grayscale
	Mat prevgray, gray;
	cvtColor(frame.img1, prevgray, CV_RGBA2GRAY);
	cvtColor(frame.img2, gray, CV_RGBA2GRAY);
	GaussianBlur(prevgray, prevgray, Size(5, 5), 0, 0);
	GaussianBlur(gray, gray, Size(5, 5), 0, 0);
	vector<KeyPoint> left_keypoints, right_keypoints;
	left_keypoints = frame.imgpts1;
	right_keypoints = frame.imgpts2;
	vector<Point2f> left_points;
	KeyPointsToPoints(left_keypoints, left_points);
	vector<Point2f> right_points(left_points.size());
	// Calculate the optical flow field:
	// how each left_point moved across the 2 images
	vector<uchar> vstatus;
	vector<float> verror;
	TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
	calcOpticalFlowPyrLK(prevgray, gray, left_points, right_points, vstatus,
		verror);	//, Size(31, 31), 8, termcrit, 0, 0.001);
	// First, filter out the points with high error
	vector<Point2f> right_points_to_find;
	vector<int> right_points_to_find_back_index;
	for (unsigned int i = 0; i < vstatus.size(); i++) {
		if (vstatus[i] && verror[i] < 12.0) {
			// Keep the original index of the point in the
			// optical flow array, for future use
			right_points_to_find_back_index.push_back(i);
			// Keep the feature point itself
			right_points_to_find.push_back(right_points[i]);
		} else {
			vstatus[i] = 0; // a bad flow
		}

	}
	// for each right_point see which detected feature it belongs to
	Mat right_points_to_find_flat = Mat(right_points_to_find).reshape(1,
		right_points_to_find.size()); //flatten array ????
	vector<Point2f> right_features; // detected features
	KeyPointsToPoints(right_keypoints, right_features);
	Mat right_features_flat = Mat(right_features).reshape(1,
		right_features.size());
	// Look around each OF point in the right image
	// for any features that were detected in its area
	// and make a match.
	BFMatcher matcher(CV_L2);
	vector<vector<DMatch> > nearest_neighbors;
	vector<DMatch> matches;
	matcher.radiusMatch(right_points_to_find_flat, right_features_flat,
		nearest_neighbors, 2.0f);
	// Check that the found neighbors are unique (throw away neighbors
	// that are too close together, as they may be confusing)
	std::set<int> found_in_right_points; // for duplicate prevention
	for (int i = 0; i < nearest_neighbors.size(); i++) {
		DMatch _m;
		if (nearest_neighbors[i].size() == 1) {
			_m = nearest_neighbors[i][0]; // only one neighbor
		} else if (nearest_neighbors[i].size() > 1) {
			// 2 neighbors – check how close they are
			double ratio = nearest_neighbors[i][0].distance
				/ nearest_neighbors[i][1].distance;
			if (ratio < 0.7) { // not too close
				// take the closest (first) one
				_m = nearest_neighbors[i][0];
			} else { // too close – we cannot tell which is better
				continue; // did not pass ratio test – throw away
			}
		} else {
			continue; // no neighbors... :(
		}
		// prevent duplicates
		if (found_in_right_points.find(_m.trainIdx)
			== found_in_right_points.end()) {
				// The found neighbor was not yet used:
				// We should match it with the original indexing
				// ofthe left point
				_m.queryIdx = right_points_to_find_back_index[_m.queryIdx];
				matches.push_back(_m); // add this match
				found_in_right_points.insert(_m.trainIdx);
		}
	}
	for (unsigned int i = 0; i < matches.size(); i++) {
		// queryIdx is the "left" image
		frame.matchedIndex1.push_back(matches[i].queryIdx);
		frame.matchPts1.push_back(left_keypoints[matches[i].queryIdx].pt);
		// trainIdx is the "right" image
		frame.matchedIndex2.push_back(matches[i].trainIdx);
		frame.matchPts2.push_back(right_keypoints[matches[i].trainIdx].pt);
	}
}




