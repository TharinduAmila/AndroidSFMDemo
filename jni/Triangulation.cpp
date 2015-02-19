#include <headers/Mtriangulation.h>

using namespace cv;
using namespace std;

Mat_<double> nViewLinearLSTriangulation(vector<Point3d> pts,
		vector<Matx34d> poses) {
	//build A & B matrix

	Mat_<double> A = Mat_<double>(
			Matx23d(pts[0].x * poses[0](2, 0) - poses[0](0, 0),
					pts[0].x * poses[0](2, 1) - poses[0](0, 1),
					pts[0].x * poses[0](2, 2) - poses[0](0, 2),
					pts[0].y * poses[0](2, 0) - poses[0](1, 0),
					pts[0].y * poses[0](2, 1) - poses[0](1, 1),
					pts[0].y * poses[0](2, 2) - poses[0](1, 2)));
	Mat_<double> B = Mat_<double>(
			Matx21d(-(pts[0].x * poses[0](2, 3) - poses[0](0, 3)),
					-(pts[0].y * poses[0](2, 3) - poses[0](1, 3))));
	for (int i = 1; i < pts.size(); i++) {
		Mat_<double> tempA = Mat_<double>(
				Matx23d(pts[i].x * poses[i](2, 0) - poses[i](0, 0),
						pts[i].x * poses[i](2, 1) - poses[i](0, 1),
						pts[i].x * poses[i](2, 2) - poses[i](0, 2),
						pts[i].y * poses[i](2, 0) - poses[i](1, 0),
						pts[i].y * poses[i](2, 1) - poses[i](1, 1),
						pts[i].y * poses[i](2, 2) - poses[i](1, 2)));
		Mat_<double> tempB = Mat_<double>(
				Matx21d(-(pts[i].x * poses[i](2, 3) - poses[i](0, 3)),
						-(pts[i].y * poses[i](2, 3) - poses[i](1, 3))));
		Mat_<double> holderA, holderB;
		vconcat(A, tempA, holderA);
		vconcat(B, tempB, holderB);
		A = holderA;
		B = holderB;
	}
	//solve the system
	Mat_<double> X;
	solve(A, B, X, DECOMP_SVD);
	return X;
}

Mat_<double> nViewIterativeLinearLSTriangulation(vector<Point3d> pts,vector<Matx34d> poses) {
	vector<double> weightArray;

	Mat_<double> X(4, 1);
	for (int i = 0; i < poses.size(); i++) {
		weightArray.push_back(1);
	}
	Mat_<double> X_ = nViewLinearLSTriangulation(pts, poses);
	X(0) = X_(0);
	X(1) = X_(1);
	X(2) = X_(2);
	X(3) = 1.0;
	for (int j = 0; j < 10; j++) {
		vector<double> p2xi;
		for (int i = 0; i < poses.size(); i++) {
			p2xi.push_back(Mat_<double>(Mat_<double>(poses[i]).row(2) * X)(0));
		}
		bool cond = true;
		for (int i = 0; i < poses.size(); i++) {
			cond = cond && fabsf(weightArray[i] - p2xi[i]) <= EPSILON;
		}
		//breaking point
		if (cond)
			break;
		for (int i = 0; i < poses.size(); i++) {
			weightArray[i] = p2xi[i];
		}

		//reweight equations and solve
		Mat_<double> A, B;
		A = Mat_<double>(
				Matx23d(
						(pts[0].x * poses[0](2, 0) - poses[0](0, 0))
								/ weightArray[0],
						(pts[0].x * poses[0](2, 1) - poses[0](0, 1))
								/ weightArray[0],
						(pts[0].x * poses[0](2, 2) - poses[0](0, 2))
								/ weightArray[0],
						(pts[0].y * poses[0](2, 0) - poses[0](1, 0))
								/ weightArray[0],
						(pts[0].y * poses[0](2, 1) - poses[0](1, 1))
								/ weightArray[0],
						(pts[0].y * poses[0](2, 2) - poses[0](1, 2))
								/ weightArray[0]));
		B = Mat_<double>(
				Matx21d(
						(-(pts[0].x * poses[0](2, 3) - poses[0](0, 3)))
								/ weightArray[0],
						(-(pts[0].y * poses[0](2, 3) - poses[0](1, 3)))
								/ weightArray[0]));
		for (int i = 1; i < pts.size(); i++) {
			Mat_<double> tempA = Mat_<double>(
					Matx23d(
							(pts[i].x * poses[i](2, 0) - poses[i](0, 0))
									/ weightArray[i],
							(pts[i].x * poses[i](2, 1) - poses[i](0, 1))
									/ weightArray[i],
							(pts[i].x * poses[i](2, 2) - poses[i](0, 2))
									/ weightArray[i],
							(pts[i].y * poses[i](2, 0) - poses[i](1, 0))
									/ weightArray[i],
							(pts[i].y * poses[i](2, 1) - poses[i](1, 1))
									/ weightArray[i],
							(pts[i].y * poses[i](2, 2) - poses[i](1, 2))
									/ weightArray[i]));
			Mat_<double> tempB = Mat_<double>(
					Matx21d(
							(-(pts[i].x * poses[i](2, 3) - poses[i](0, 3)))
									/ weightArray[i],
							(-(pts[i].y * poses[i](2, 3) - poses[i](1, 3)))
									/ weightArray[i]));
			Mat_<double> holderA, holderB;
			vconcat(A, tempA, holderA);
			vconcat(B, tempB, holderB);
			A = holderA;
			B = holderB;
		}
		solve(A, B, X_, DECOMP_SVD);
		X(0) = X_(0);
		X(1) = X_(1);
		X(2) = X_(2);
		X(3) = 1.0;
	}
	return X;
}

double TriangulatePoints(const vector<KeyPoint>& pt_set1,
		const vector<KeyPoint>& pt_set2, const Mat& K, const Mat&Kinv,
		const Matx34d& P, const Matx34d& P1, vector<Point3d>& pointcloud) {
	vector<double> reproj_error;
	pointcloud.clear();
	for (unsigned int i = 0; i < pt_set1.size(); i++) {
		//convert to normalized homogeneous coordinates
		Point2f kp = pt_set1[i].pt;
		Point3d u(kp.x, kp.y, 1.0);
		Mat_<double> um = Kinv * Mat_<double>(u);
		u = Point3d(um.at<double>(0, 0), um.at<double>(1, 0),
				um.at<double>(2, 0));
		Point2f kp1 = pt_set2[i].pt;
		Point3d u1(kp1.x, kp1.y, 1.0);
		Mat_<double> um1 = Kinv * Mat_<double>(u1);
		//u1 = um1.at<Point3d>(0);
		u1 = Point3d(um1.at<double>(0, 0), um1.at<double>(1, 0),
				um1.at<double>(2, 0));
		vector<Point3d> pts;
		vector<Matx34d> poses;
		pts.push_back(u);
		pts.push_back(u1);
		poses.push_back(P);
		poses.push_back(P1);

		//triangulate
		Mat_<double> X = nViewIterativeLinearLSTriangulation(pts,poses); //LinearLSTriangulation(u, u1,P,P1);
		//calculate reprojection error
		Mat_<double> xPt_img = K * Mat(P1) * X;
		Point2f xPt_img_(xPt_img(0) / xPt_img(2), xPt_img(1) / xPt_img(2));
		reproj_error.push_back(norm(xPt_img_ - kp1));
		//store 3D point
		pointcloud.push_back(Point3d(X(0), X(1), X(2)));
	}
	//return mean reprojection error
	Scalar me = mean(reproj_error);
	return me[0];
}
void TriangulateGraphPoints(MGraph& graph){
	map<int,worldData>::iterator it = graph.pointMap.begin();
	for(;it!=graph.pointMap.end();it++){
		vector<Point3d> pts;
		vector<Matx34d> poses;
		if(!it->second.track.empty()){
			for(int i=0;i<it->second.track.size();i++){
				//convert to normalized homogeneous coordinates
				if(it->second.track[i]!=-1){
				Point2f kp = graph.obsValues[i][it->second.track[i]];
				Point3d u(kp.x, kp.y, 1.0);
				Mat_<double> um = graph.cameraMat.inv() * Mat_<double>(u);
				u = Point3d(um.at<double>(0,0),um.at<double>(1,0),um.at<double>(2,0));
				pts.push_back(u);
				poses.push_back(graph.relativePoses[i]);
				}
			}
		}
		Mat_<double> newPoint = nViewIterativeLinearLSTriangulation(pts,poses);
		Point3d X(newPoint(0),newPoint(1),newPoint(2));
		it->second.point = X;
	}
}
