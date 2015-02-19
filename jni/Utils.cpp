#include <headers/Mutils.h>

using namespace cv;
using namespace std;

void createPoseFromRotationTranslation(Mat_<double> R, Mat_<double> t,
		Matx34d& Pose) {
	Pose = Matx34d(R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0), R(1, 1), R(1, 2),
			t(1), R(2, 0), R(2, 1), R(2, 2), t(2));
}

void KeyPointsToPoints(vector<KeyPoint> in, vector<Point2f>& out) {
	out.clear();
	for (int i = 0; i < in.size(); i++) {
		out.push_back(in[i].pt);
	}
}
void KeyPointsToPoints(vector<KeyPoint> in, vector<Point2d>& out) {
	out.clear();
	for (int i = 0; i < in.size(); i++) {
		out.push_back(in[i].pt);
	}
}
void PointsToKeyPoints(vector<Point2f> in, vector<KeyPoint>& out) {
	out.clear();
	for (int i = 0; i < in.size(); i++) {
		out.push_back(KeyPoint(in[i], CV_32F));
	}
}
void PointsToKeyPoints(vector<Point2d> in, vector<KeyPoint>& out) {
	out.clear();
	for (int i = 0; i < in.size(); i++) {
		out.push_back(KeyPoint(in[i], CV_32F));
	}
}
void Point2fToPoint2d(std::vector<cv::Point2f> in, std::vector<cv::Point2d>& out){
	out.clear();
	for (int i = 0; i < in.size(); i++) {
		out.push_back(Point2d(in[i].x,in[i].y));
	}
}
void putTextOnTopOfMat(Mat & in,string print,double scale,int thickness){
	int fontFace = FONT_HERSHEY_PLAIN;
	int baseline = 0;
	Size textSize = getTextSize(print, fontFace, scale, thickness,&baseline);
	putText(in, print, Point(0, textSize.height), fontFace, scale,Scalar::all(255), thickness, 8);

}
string convertIntToString(int in){
	std::ostringstream ss;
	ss << in;
	return ss.str();
}
void findEssentialMatrix(MFramePair& pair, Mat_<double> K) {
			vector<Point2f> k1, k2,n1,n2;
			vector<int> usedIndex1,usedIndex2;
			vector<uchar> status(pair.imgpts1.size());
			Mat F = findFundamentalMat(pair.matchPts1, pair.matchPts2, CV_FM_RANSAC, 0.2, 0.9,
				status);
			Mat_<double> E;
			E = K.t() * F * K; //according to HZ (9.12)
			for (unsigned int i = 0; i < status.size(); i++) { // queryIdx is the "left" image
				if (status[i]) {
					usedIndex1.push_back(pair.matchedIndex1[i]);
					k1.push_back(pair.matchPts1[i]);
					usedIndex2.push_back(pair.matchedIndex2[i]);
					k2.push_back(pair.matchPts2[i]);
				}
			}
			correctMatches(F,k1,k2,n1,n2);
			pair.matchPts1 = n1;
			pair.matchPts2 = n2;
			pair.matchedIndex1=usedIndex1;
			pair.matchedIndex2=usedIndex2;
			pair.F = F;
			pair.E = E;
}
bool findCorrectPose(Mat_<double> E, vector<KeyPoint> key1, vector<KeyPoint> key2, Mat_<double> K,Mat_<double> Kinv,Matx34d& out,vector<Point3d>& cloudOut) {
	Matx34d Peye(1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0);
	//decompose E to P' , HZ (9.19)
			cv::SVD svd(E, SVD::MODIFY_A);
			Mat svd_u = svd.u;
			Mat svd_vt = svd.vt;
			Mat svd_w = svd.w;
			Matx33d W(0, -1, 0,	//HZ 9.13
				1, 0, 0, 0, 0, 1);
	/////////////////////////////////////////////////////

	//newly added must check if correct
	Matx33d Z(0, 1, 0,	//HZ 9.13
		-1, 0, 0, 0, 0, 0);
	Mat_<double> S = svd_u * Mat(Z) * svd_u.t();
	///////////////////////////////////
	Mat_<double> R = svd_u * Mat(W) * svd_vt; //HZ 9.19
	Mat_<double> R1 = svd_u * Mat(W).t() * svd_vt; //HZ 9.19
	///////////////////newly added according to mit implementation////////
	if(determinant(R)<0)
		R = -R;
	if(determinant(R1)<0)
		R1 = -R1;
	/////////////////////////////////////
	Mat_<double> t = svd_u.col(2); // u3
	Mat_<double> tneg = -1 * svd_u.col(2); // -u3
	cv::SVD rSVD(R);
	R = rSVD.u * Mat::eye(3, 3, CV_64F) * rSVD.vt;
	double myscale = trace(rSVD.w)[0] / 3;
	t = t / myscale;
	cv::SVD r1SVD(R1);
	R1 = r1SVD.u * Mat::eye(3, 3, CV_64F) * r1SVD.vt;
	double myscale1 = trace(r1SVD.w)[0] / 3;
	tneg = tneg / myscale1;
	if (!CheckCoherentRotation(R) || !CheckCoherentRotation(R1)) {
		out = 0;
		return false;
	}
	//save the correct pointclouds and Relativepose
	Matx34d TempPose,poseHolder;
	vector<Point3d> pointCloud,pointCloudMax;

	createPoseFromRotationTranslation(R, t, out);
	double max = -1,temp = 0.0;
	max = getPointCloudAndPointsInFront(Peye, out, key1, key2, K, K.inv(), pointCloud);
	pointCloudMax = pointCloud;
	poseHolder = Matx34d(out);


	pointCloud.clear();
	out = TempPose;
	createPoseFromRotationTranslation(R, tneg, out);
	temp = getPointCloudAndPointsInFront(Peye, out, key1, key2, K, K.inv(), pointCloud);
	if(temp>max){
		pointCloudMax = pointCloud;
		poseHolder = Matx34d(out);
		max= temp;
	}

	pointCloud.clear();
	out = TempPose;
	createPoseFromRotationTranslation(R1, t, out);
	temp = getPointCloudAndPointsInFront(Peye, out, key1, key2, K, K.inv(), pointCloud);
	if(temp>max){
		pointCloudMax = pointCloud;
		poseHolder = Matx34d(out);
		max= temp;
	}

	pointCloud.clear();
	out = TempPose;
	createPoseFromRotationTranslation(R1, tneg, out);
	temp = getPointCloudAndPointsInFront(Peye, out, key1, key2, K, K.inv(), pointCloud);
	if(temp>max){
		pointCloudMax = pointCloud;
		poseHolder = Matx34d(out);
		max= temp;
	}
	out = poseHolder;
	cloudOut = pointCloudMax;
	return true;
}
bool CheckCoherentRotation(cv::Mat_<double>& R) {
	if (fabsf(determinant(R)) - 1.0 > 1e-07) {
		return false;
	}
	return true;
}
double getPointCloudAndPointsInFront(Matx34d P, Matx34d Pose,
		vector<KeyPoint> key1, vector<KeyPoint> key2, Mat_<double> K,
		Mat_<double> Kinv, vector<Point3d>& pointCloud) {
								vector<Point3d> pcloud_pt3d;
								double error=0;
								//testTriangulation Process
								error = TriangulatePoints(key1, key2, K, Kinv, P, Pose, pcloud_pt3d);
								//cout << "Tri Error " << error << endl;
								pointCloud = pcloud_pt3d;
								vector<Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());

								int count1 = 0;
								for (int i = 0; i < pointCloud.size(); i++) {
									count1 += (pointCloud[i].z > 0) ? 1 : 0;
								}

								Matx44d P4x4 = Matx44d::eye();
								for (int i = 0; i < 12; i++)
									P4x4.val[i] = Pose.val[i];

								perspectiveTransform(pcloud_pt3d, pcloud_pt3d_projected, P4x4);
								int count = 0;
								for (int i = 0; i < pcloud_pt3d_projected.size(); i++) {
									count += (pcloud_pt3d_projected[i].z > 0) ? 1 : 0;
								}
								double percentage = ((double) (count + count1)
									/ (double) (pointCloud.size() * 2));
								return percentage;
}
void getNegativePose(Matx34d poseIn, Matx34d& poseOut) {
	Matx33d R(poseIn(0, 0), poseIn(0, 1), poseIn(0, 2), poseIn(1, 0),
			poseIn(1, 1), poseIn(1, 2), poseIn(2, 0), poseIn(2, 1),
			poseIn(2, 2));
	Matx33d Rinv;
	Rinv = R.t(); //or transpose as Rotation matrices are orthonormal
	Matx31d t(poseIn(0, 3), poseIn(1, 3), poseIn(2, 3));
	Matx31d negt;
	negt = Rinv * (-t);
	Matx34d out(Rinv(0, 0), Rinv(0, 1), Rinv(0, 2), negt(0), Rinv(1, 0),
			Rinv(1, 1), Rinv(1, 2), negt(1), Rinv(2, 0), Rinv(2, 1), Rinv(2, 2),
			negt(2));
	poseOut = out;
}
void extractRfromePose(Matx34d relativePose,Mat_<double>& out){
	out = (cv::Mat_<double>(3,3)<< relativePose(0,0), relativePose(0,1),relativePose(0,2),
									 relativePose(1,0), relativePose(1,1), relativePose(1,2),
									 relativePose(2,0), relativePose(2,1), relativePose(2,2));
}
void extractTfromePose(Matx34d relativePose,Mat_<double>& out){
	out = (cv::Mat_<double>(3,1) << relativePose(0,3), relativePose(1,3), relativePose(2,3));
}

void concatenateRts(Matx34d Rtouter,Matx34d Rtinner,Matx34d& out){
	Mat_<double> R1,R2,T1,T2,R0,T0;
	extractRfromePose(Rtouter,R1);
	extractRfromePose(Rtinner,R2);
	extractTfromePose(Rtouter,T1);
	extractTfromePose(Rtinner,T2);
	R0 = R1 * R2;
	T0 = R1 * T2 + T1;
	createPoseFromRotationTranslation(R0,T0,out);
}



void mergePointCloudBtoA(MGraph& A,MGraph& B){
	Matx34d negPoseFromA;
	A.framesIncluded.push_back(B.framesIncluded[1]);
	A.obsValues.push_back(B.obsValues[B.obsValues.size()-1]);
	getNegativePose(A.relativePoses[A.relativePoses.size()-1],negPoseFromA);
	Matx34d RtBW2AW,invRtBW2AW;
	concatenateRts(negPoseFromA,B.relativePoses[0],RtBW2AW);
	//convert points in B to frame of A
	Matx44d P4x4 = Matx44d::eye();
	for (int i = 0; i < 12; i++)
		P4x4.val[i] = RtBW2AW.val[i];
	vector<Point3d> tempPointCloudHolder;
	perspectiveTransform(B.pointCloud, tempPointCloudHolder, P4x4);
	B.pointCloud = tempPointCloudHolder;
	///////////////pose addition/////////////
	getNegativePose(RtBW2AW,invRtBW2AW);
	for (int i=0;i<B.relativePoses.size();i++){
		Matx34d temp;
		concatenateRts(B.relativePoses[i], invRtBW2AW,temp);
		B.relativePoses[i] = temp;
	}
	A.relativePoses.push_back(B.relativePoses[B.relativePoses.size()-1]);
}
void addPointClouds(MGraph& a,MGraph b){
	int counter = -1;
	if(a.pointMap.empty()){
			a.framesIncluded.push_back(b.framesIncluded[0]);
			a.framesIncluded.push_back(b.framesIncluded[1]);
			a.relativePoses.push_back(b.relativePoses[0]);
			a.relativePoses.push_back(b.relativePoses[1]);
		for(int i=0;i<b.pointCloud.size();i++){
			worldData temp;
			temp.point = b.pointCloud[i];
			temp.track = b.obsIndexes[i];
			a.obsValues = b.obsValues;
			a.pointMap[b.obsIndexes[i][1]] = temp;
		}
	}else{
		a.framesIncluded.push_back(b.framesIncluded[1]);
		a.relativePoses.push_back(b.relativePoses[1]);
		map<int, worldData> comMap;
		for(int i=0;i<b.pointCloud.size();i++){
			worldData temp;
			//map<int,worldData>::iterator it = a.pointMap.find(b.obsIndexes[i][0]);
			if(a.pointMap.find(b.obsIndexes[i][0])!=a.pointMap.end()){
				temp = a.pointMap[b.obsIndexes[i][0]];
				temp.track.push_back(b.obsIndexes[i][1]);
				comMap[b.obsIndexes[i][1]] = temp;
				a.pointMap[b.obsIndexes[i][0]].track.clear();
			}else{
				//toDo
				for(int j=0;j<(a.pointMap.begin()->second.track.size()-1);j++)
					temp.track.push_back(-1);
				temp.track.push_back(b.obsIndexes[i][0]);
				temp.track.push_back(b.obsIndexes[i][1]);
				temp.point = b.pointCloud[i];
				comMap[b.obsIndexes[i][1]] = temp;
			}
		}
		map<int,worldData>::iterator it = a.pointMap.begin();
		for(;it!=a.pointMap.end();it++){
			if(!it->second.track.empty()){
				worldData temp = it->second;
				temp.track.push_back(-1);
				comMap[counter] = temp;
				counter--;
			}
		}
		a.pointMap = comMap;
	}

}
void removeOutliers(MGraph& in){
	map<int,worldData>::iterator it = in.pointMap.begin();
	in.pointCloud.clear();
	map<int,worldData> filteredMap;
	Mat_<double> K = in.cameraMat;
	Mat_<double> X(4, 1);
	double error;
	int count,count1=0;
	for(;it!=in.pointMap.end();it++){
		error = 0;
		count = 0;
		X(0) = it->second.point.x;
		X(1) = it->second.point.y;
		X(2) = it->second.point.z;
		X(3) = 1.0;
		for(int i=0;i<it->second.track.size();i++){
			if(it->second.track[i]!=-1){
				Point2f obsPoint = in.obsValues[i][it->second.track[i]];
				Mat_<double> P = Mat_<double>(in.relativePoses[i]);
				error += getReprojectionError(K,P,X,obsPoint);
				count++;
			}
		}
		if((error/count)<25 && X(2)>-1){
			filteredMap[it->first] = it->second;
			in.pointCloud.push_back(it->second.point);
		}else{
			string print =  "removed "+convertIntToString(count1++) +"\n";
			__android_log_write(ANDROID_LOG_DEBUG, "OutLier Removal", print.c_str());
		}
	}
	in.pointMap = filteredMap;
}
double getReprojectionError(Mat_<double> K,Mat_<double> P1,Mat_<double> X,Point2f kp1){
	Mat_<double> xPt_img = K * Mat(P1) * X;
	Point2f xPt_img_(xPt_img(0) / xPt_img(2), xPt_img(1) / xPt_img(2));
	return norm(xPt_img_ - kp1);
}

MGraph mergePointClouds(vector<MGraph> graphList){
	MGraph out;
	for(int i=1;i<graphList.size();i++){
		mergePointCloudBtoA(graphList[0],graphList[i]);
	}
	out.cameraMat = graphList[0].cameraMat;
	for(int i=0;i<graphList.size();i++){
		addPointClouds(out,graphList[i]);
		TriangulateGraphPoints(out);
		graphBundleAdjustment(out);
	}
	return out;
}
