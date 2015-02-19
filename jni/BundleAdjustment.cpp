#include <headers/MbundleAdjustment.h>
#include <headers/cvsba.h>

using namespace cv;
using namespace std;

static void fjac(int /*i*/, int /*j*/, CvMat *point_params, CvMat* cam_params,
		CvMat* A, CvMat* B, void* /*data*/) {
	//compute jacobian per camera parameters (i.e. Aij)
	//take i-th point 3D current coordinates

	CvMat _Mi;
	cvReshape(point_params, &_Mi, 3, 1);

	CvMat* _mp = cvCreateMat(1, 1, CV_64FC2); //projection of the point

	//split camera params into different matrices
	CvMat _ri, _ti, _k = cvMat(0, 0, CV_64F, NULL); // dummy initialization to fix warning of cl.exe
	cvGetRows(cam_params, &_ri, 0, 3);
	cvGetRows(cam_params, &_ti, 3, 6);

	double intr_data[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 1 };
	intr_data[0] = cam_params->data.db[6];
	intr_data[4] = cam_params->data.db[7];
	intr_data[2] = cam_params->data.db[8];
	intr_data[5] = cam_params->data.db[9];

	CvMat _A = cvMat(3, 3, CV_64F, intr_data);

	CvMat _dpdr, _dpdt, _dpdf, _dpdc, _dpdk;

	bool have_dk = cam_params->height - 10 ? true : false;

	cvGetCols(A, &_dpdr, 0, 3);
	cvGetCols(A, &_dpdt, 3, 6);
	cvGetCols(A, &_dpdf, 6, 8);
	cvGetCols(A, &_dpdc, 8, 10);

	if (have_dk) {
		cvGetRows(cam_params, &_k, 10, cam_params->height);
		cvGetCols(A, &_dpdk, 10, A->width);
	}
	cvProjectPoints2(&_Mi, &_ri, &_ti, &_A, have_dk ? &_k : NULL, _mp, &_dpdr,
			&_dpdt, &_dpdf, &_dpdc, have_dk ? &_dpdk : NULL, 0);

	cvReleaseMat(&_mp);

	//get rotation matrix
	double R[9], t[3], fx = intr_data[0], fy = intr_data[4];
	CvMat _R = cvMat(3, 3, CV_64F, R);
	cvRodrigues2(&_ri, &_R);

	double X, Y, Z;
	X = point_params->data.db[0];
	Y = point_params->data.db[1];
	Z = point_params->data.db[2];

	t[0] = _ti.data.db[0];
	t[1] = _ti.data.db[1];
	t[2] = _ti.data.db[2];

	//compute x,y,z
	double x = R[0] * X + R[1] * Y + R[2] * Z + t[0];
	double y = R[3] * X + R[4] * Y + R[5] * Z + t[1];
	double z = R[6] * X + R[7] * Y + R[8] * Z + t[2];

#if 1
	//compute x',y'
	double x_strike = x / z;
	double y_strike = y / z;
	//compute dx',dy'  matrix
	//
	//    dx'/dX  dx'/dY dx'/dZ    =
	//    dy'/dX  dy'/dY dy'/dZ

	double coeff[6] = { z, 0, -x, 0, z, -y };
	CvMat coeffmat = cvMat(2, 3, CV_64F, coeff);

	CvMat* dstrike_dbig = cvCreateMat(2, 3, CV_64F);
	cvMatMul(&coeffmat, &_R, dstrike_dbig);
	cvScale(dstrike_dbig, dstrike_dbig, 1 / (z * z));

	if (have_dk) {
		double strike_[2] = { x_strike, y_strike };
		CvMat strike = cvMat(1, 2, CV_64F, strike_);

		//compute r_2
		double r_2 = x_strike * x_strike + y_strike * y_strike;
		double r_4 = r_2 * r_2;
		double r_6 = r_4 * r_2;

		//compute d(r_2)/dbig
		CvMat* dr2_dbig = cvCreateMat(1, 3, CV_64F);
		cvMatMul(&strike, dstrike_dbig, dr2_dbig);
		cvScale(dr2_dbig, dr2_dbig, 2);

		double& k1 = _k.data.db[0];
		double& k2 = _k.data.db[1];
		double& p1 = _k.data.db[2];
		double& p2 = _k.data.db[3];
		double k3 = 0;

		if (_k.cols * _k.rows == 5) {
			k3 = _k.data.db[4];
		}
		//compute dg/dbig
		double dg_dr2 = k1 + k2 * 2 * r_2 + k3 * 3 * r_4;
		double g = 1 + k1 * r_2 + k2 * r_4 + k3 * r_6;

		CvMat* dg_dbig = cvCreateMat(1, 3, CV_64F);
		cvScale(dr2_dbig, dg_dbig, dg_dr2);

		CvMat* tmp = cvCreateMat(2, 3, CV_64F);
		CvMat* dstrike2_dbig = cvCreateMat(2, 3, CV_64F);

		double c[4] = { g + 2 * p1 * y_strike + 4 * p2 * x_strike, 2 * p1
				* x_strike, 2 * p2 * y_strike, g + 2 * p2 * x_strike
				+ 4 * p1 * y_strike };

		CvMat coeffmat2 = cvMat(2, 2, CV_64F, c);

		cvMatMul(&coeffmat2, dstrike_dbig, dstrike2_dbig);

		cvGEMM(&strike, dg_dbig, 1, NULL, 0, tmp, CV_GEMM_A_T);
		cvAdd(dstrike2_dbig, tmp, dstrike2_dbig);

		double p[2] = { p2, p1 };
		CvMat pmat = cvMat(2, 1, CV_64F, p);

		cvMatMul(&pmat, dr2_dbig, tmp);
		cvAdd(dstrike2_dbig, tmp, dstrike2_dbig);

		cvCopy(dstrike2_dbig, B);

		cvReleaseMat(&dr2_dbig);
		cvReleaseMat(&dg_dbig);

		cvReleaseMat(&tmp);
		cvReleaseMat(&dstrike2_dbig);
		cvReleaseMat(&tmp);
	} else {
		cvCopy(dstrike_dbig, B);
	}
	//multiply by fx, fy
	CvMat row;
	cvGetRows(B, &row, 0, 1);
	cvScale(&row, &row, fx);

	cvGetRows(B, &row, 1, 2);
	cvScale(&row, &row, fy);

#else

	double k = fx/(z*z);

	cvmSet( B, 0, 0, k*(R[0]*z-x*R[6]));
	cvmSet( B, 0, 1, k*(R[1]*z-x*R[7]));
	cvmSet( B, 0, 2, k*(R[2]*z-x*R[8]));

	k = fy/(z*z);

	cvmSet( B, 1, 0, k*(R[3]*z-y*R[6]));
	cvmSet( B, 1, 1, k*(R[4]*z-y*R[7]));
	cvmSet( B, 1, 2, k*(R[5]*z-y*R[8]));

#endif

}
static void func(int /*i*/, int /*j*/, CvMat *point_params, CvMat* cam_params,
		CvMat* estim, void* /*data*/) {
	//just do projections
	CvMat _Mi;
	cvReshape(point_params, &_Mi, 3, 1);

	CvMat* _mp = cvCreateMat(1, 1, CV_64FC2); //projection of the point
	CvMat* _mp2 = cvCreateMat(1, 2, CV_64F); //projection of the point

	//split camera params into different matrices
	CvMat _ri, _ti, _k;

	cvGetRows(cam_params, &_ri, 0, 3);
	cvGetRows(cam_params, &_ti, 3, 6);

	double intr_data[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 1 };
	intr_data[0] = cam_params->data.db[6];
	intr_data[4] = cam_params->data.db[7];
	intr_data[2] = cam_params->data.db[8];
	intr_data[5] = cam_params->data.db[9];

	CvMat _A = cvMat(3, 3, CV_64F, intr_data);

	//int cn = CV_MAT_CN(_Mi.type);

	bool have_dk = cam_params->height - 10 ? true : false;

	if (have_dk) {
		cvGetRows(cam_params, &_k, 10, cam_params->height);
	}
	cvProjectPoints2(&_Mi, &_ri, &_ti, &_A, have_dk ? &_k : NULL, _mp, NULL,
	NULL,
	NULL, NULL, NULL, 0);
	//    std::cerr<<"_mp = "<<_mp->data.db[0]<<","<<_mp->data.db[1]<<std::endl;
	//
	_mp2->data.db[0] = _mp->data.db[0];
	_mp2->data.db[1] = _mp->data.db[1];
	cvTranspose(_mp2, estim);
	cvReleaseMat(&_mp);
	cvReleaseMat(&_mp2);
}
static void fjac_new(int i, int j, Mat& point_params, Mat& cam_params, Mat& A,
		Mat& B, void* data) {
	CvMat _point_params = point_params, _cam_params = cam_params, _Al = A, _Bl =
			B;
	fjac(i, j, &_point_params, &_cam_params, &_Al, &_Bl, data);
}
static void func_new(int i, int j, Mat& point_params, Mat& cam_params,
		Mat& estim, void* data) {
	CvMat _point_params = point_params, _cam_params = cam_params,
			_estim = estim;
	func(i, j, &_point_params, &_cam_params, &_estim, data);
}
void bundleAdjust(
		vector<Point3d>& points, //positions of points in global coordinate system (input and output)
		const vector<vector<Point2d> >& imagePoints, //projections of 3d points for every camera
		const vector<vector<int> >& visibility, //visibility of 3d points for every camera
		vector<Mat>& cameraMatrix, //intrinsic matrices of all cameras (input and output)
		vector<Mat>& R, //rotation matrices of all cameras (input and output)
		vector<Mat>& T, //translation vector of all cameras (input and output)
		vector<Mat>& distCoeffs, //distortion coefficients of all cameras (input and output)
		const TermCriteria& criteria, BundleAdjustCallback cb,
		void* user_data) {
	//,enum{MOTION_AND_STRUCTURE,MOTION,STRUCTURE})
	int num_points = (int) points.size();
	int num_cameras = (int) cameraMatrix.size();

	CV_Assert(
			imagePoints.size() == (size_t )num_cameras
					&& visibility.size() == (size_t )num_cameras
					&& R.size() == (size_t )num_cameras
					&& T.size() == (size_t )num_cameras
					&& (distCoeffs.size() == (size_t )num_cameras
							|| distCoeffs.size() == 0));

	int numdist =
			distCoeffs.size() ? (distCoeffs[0].rows * distCoeffs[0].cols) : 0;

	int num_cam_param = 3 /* rotation vector */+ 3 /* translation vector */
	+ 2 /* fx, fy */+ 2 /* cx, cy */+ numdist;

	int num_point_param = 3;

	//collect camera parameters into vector
	Mat params(num_cameras * num_cam_param + num_points * num_point_param, 1,
	CV_64F);

	//fill camera params
	for (int i = 0; i < num_cameras; i++) {
		//rotation
		Mat rot_vec;
		Rodrigues(R[i], rot_vec);
		Mat dst = params.rowRange(i * num_cam_param, i * num_cam_param + 3);
		rot_vec.copyTo(dst);

		//translation
		dst = params.rowRange(i * num_cam_param + 3, i * num_cam_param + 6);
		T[i].copyTo(dst);

		//intrinsic camera matrix
		double* intr_data = (double*) cameraMatrix[i].data;
		double* intr = (double*) (params.data
				+ params.step * (i * num_cam_param + 6));
		//focals
		intr[0] = intr_data[0];  //fx
		intr[1] = intr_data[4];  //fy
		//center of projection
		intr[2] = intr_data[2];  //cx
		intr[3] = intr_data[5];  //cy

		//add distortion if exists
		if (distCoeffs.size()) {
			dst = params.rowRange(i * num_cam_param + 10,
					i * num_cam_param + 10 + numdist);
			distCoeffs[i].copyTo(dst);
		}
	}

	//fill point params
	Mat ptparams(num_points, 1, CV_64FC3,
			params.data + num_cameras * num_cam_param * params.step);
	Mat _points(points);
	CV_Assert(
			_points.size() == ptparams.size()
					&& _points.type() == ptparams.type());
	_points.copyTo(ptparams);

	//convert visibility vectors to visibility matrix
	Mat vismat(num_points, num_cameras, CV_32S);
	for (int i = 0; i < num_cameras; i++) {
		//get row
		Mat col = vismat.col(i);
		Mat((int) visibility[i].size(), 1, vismat.type(),
				(void*) &visibility[i][0]).copyTo(col);
	}

	int num_proj = countNonZero(vismat); //total number of points projections

	//collect measurements
	Mat X(num_proj * 2, 1, CV_64F); //measurement vector

	int counter = 0;
	for (int i = 0; i < num_points; i++) {
		for (int j = 0; j < num_cameras; j++) {
			//check visibility
			if (visibility[j][i]) {
				//extract point and put tu vector
				Point2d p = imagePoints[j][i];
				((double*) (X.data))[counter] = p.x;
				((double*) (X.data))[counter + 1] = p.y;
				assert(p.x != -1 || p.y != -1);
				counter += 2;
			}
		}
	}

	LevMarqSparse levmar(num_points, num_cameras, num_point_param,
			num_cam_param, 2, vismat, params, X, TermCriteria(criteria),
			fjac_new, func_new, NULL, cb, user_data);
	//extract results
	//fill point params
	/*Mat final_points(num_points, 1, CV_64FC3,
	 levmar.P->data.db + num_cameras*num_cam_param *levmar.P->step);
	 CV_Assert(_points.size() == final_points.size() && _points.type() == final_points.type());
	 final_points.copyTo(_points);*/

	points.clear();
	for (int i = 0; i < num_points; i++) {
		CvMat point_mat;
		cvGetSubRect(levmar.P, &point_mat,
				cvRect(0,
						levmar.num_cams * levmar.num_cam_param
								+ levmar.num_point_param * i, 1,
						levmar.num_point_param));
		CvScalar x = cvGet2D(&point_mat, 0, 0);
		CvScalar y = cvGet2D(&point_mat, 1, 0);
		CvScalar z = cvGet2D(&point_mat, 2, 0);
		points.push_back(Point3d(x.val[0], y.val[0], z.val[0]));
		//std::cerr<<"point"<<points[points.size()-1].x<<","<<points[points.size()-1].y<<","<<points[points.size()-1].z<<std::endl;
	}
	//fill camera params
	//R.clear();T.clear();cameraMatrix.clear();
	for (int i = 0; i < num_cameras; i++) {
		//rotation
		Mat rot_vec = Mat(levmar.P).rowRange(i * num_cam_param,
				i * num_cam_param + 3);
		Mat R_tmp(3, 3, CV_64F);
		Rodrigues(rot_vec, R_tmp);
		R_tmp.copyTo(R[i]);
		//translation
		// T[i] = Mat(levmar.P).rowRange(i*num_cam_param + 3, i*num_cam_param+6);
		(Mat(levmar.P).rowRange(i * num_cam_param + 3, i * num_cam_param + 6)).copyTo(
				T[i]);

		//intrinsic camera matrix
		double* intr_data = (double*) cameraMatrix[i].data;
		double* intr = (double*) (Mat(levmar.P).data
				+ Mat(levmar.P).step * (i * num_cam_param + 6));
		//focals
		intr_data[0] = intr[0];  //fx
		intr_data[4] = intr[1];  //fy
		//center of projection
		intr_data[2] = intr[2];  //cx
		intr_data[5] = intr[3];  //cy

		//add distortion if exists
		if (distCoeffs.size()) {
			Mat(levmar.P).rowRange(i * num_cam_param + 10,
					i * num_cam_param + 10 + numdist).copyTo(distCoeffs[i]);
		}
	}
}

void twoViewBundleAdjustment(
		Mat_<double> cameraMat, //intrinsic
		std::vector<cv::Point3d>& points_opt,
		vector<vector<Point2d> > matchesUsedFor3d, Matx34d relativePose,
		vector<Matx34d>& out) {
	std::vector<cv::Mat> R_opt;
	std::vector<cv::Mat> T_opt;
	std::vector<std::vector<cv::Point2d> > imagePoints;
	std::vector<std::vector<int> > visiblity;
	std::vector<cv::Mat> cameraMatrix;
	std::vector<cv::Mat> distCoeffs;
	cv::TermCriteria criteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-12);
	//add image points
	vector<int> tempint1, tempint2;
	__android_log_write(ANDROID_LOG_ERROR, "Two View", "stage1");
	for (int i = 0; i < matchesUsedFor3d[0].size(); i++) {
		tempint1.push_back(1);
		tempint2.push_back(1);
	}
	__android_log_write(ANDROID_LOG_ERROR, "Two View", "stage2");
	visiblity.push_back(tempint1);
	visiblity.push_back(tempint2);

	// define cameras
	cameraMatrix.push_back(cameraMat);
	cameraMatrix.push_back(cameraMat);

	distCoeffs.push_back((cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0));
	distCoeffs.push_back((cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0));

	R_opt.push_back(
			(cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0));
	__android_log_write(ANDROID_LOG_ERROR, "Two View", "stage3");
	R_opt.push_back(
			(cv::Mat_<double>(3, 3) << relativePose(0, 0), relativePose(0, 1), relativePose(
					0, 2), relativePose(1, 0), relativePose(1, 1), relativePose(
					1, 2), relativePose(2, 0), relativePose(2, 1), relativePose(
					2, 2)));
	__android_log_write(ANDROID_LOG_ERROR, "Two View", "stage4");
	T_opt.push_back((cv::Mat_<double>(3, 1) << 0, 0, 0));
	__android_log_write(ANDROID_LOG_ERROR, "Two View", "stage5");
	T_opt.push_back(
			(cv::Mat_<double>(3, 1) << relativePose(0, 3), relativePose(1, 3), relativePose(
					2, 3)));
	__android_log_write(ANDROID_LOG_ERROR, "Two View", "stage6");
	//make copies for visualization
	vector<cv::Point3d> points;
	std::vector<cv::Mat> Rvec, Tvec;
	points = points_opt;
	for (int k = 0; k < R_opt.size(); k++) {
		Rvec.push_back(R_opt[k].clone());
		Tvec.push_back(T_opt[k].clone());
	}
//	//trying sba

	cvsba::Sba::Params params;
	params.type = cvsba::Sba::MOTIONSTRUCTURE;
	params.iterations = 150;
	params.minError = 1e-10;
	params.fixedIntrinsics = 5;
	params.fixedDistortion = 5;
	params.verbose = false;
	cvsba::Sba sba;
	sba.setParams(params);
	__android_log_write(ANDROID_LOG_ERROR, "Two View", "stage7");

	sba.run(points_opt, matchesUsedFor3d, visiblity, cameraMatrix, R_opt, T_opt,
			distCoeffs);
	__android_log_write(ANDROID_LOG_ERROR, "Two View", "stage8");
	Matx34d P1, P2;
	createPoseFromRotationTranslation(R_opt[0], T_opt[0], P1);
	createPoseFromRotationTranslation(R_opt[1], T_opt[1], P2);
	__android_log_write(ANDROID_LOG_ERROR, "Two View", "stage9");
	out.push_back(P1);
	out.push_back(P2);
	///////////////////////////////////////////////////////////////////////////////////////////////
}
void graphBundleAdjustment(MGraph& graph) {
	vector<Point3d> points_opt;
	std::vector<cv::Mat> R_opt;
	std::vector<cv::Mat> T_opt;
	std::vector<cv::Mat> cameraMatrix;
	vector<vector<int> > visibility;
	std::vector<cv::Mat> distCoeffs;
	cv::TermCriteria criteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 80, 1e-12);
	vector<vector<Point2d> > matchesUsedFor3d;
	Mat_<double> cameraMat = graph.cameraMat;
	//extract data
	map<int, worldData>::iterator it = graph.pointMap.begin();
	for (int k = 0; k < it->second.track.size(); k++) {
		vector<Point2d> temp;
		vector<int> t;
		matchesUsedFor3d.push_back(temp);
		visibility.push_back(t);
		cameraMatrix.push_back(cameraMat);
		Mat_<double> R, T;
		extractRfromePose(graph.relativePoses[k], R);
		extractTfromePose(graph.relativePoses[k], T);
		R_opt.push_back(R);
		T_opt.push_back(T);
		distCoeffs.push_back((cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0));
	}

	for (; it != graph.pointMap.end(); it++) {
		points_opt.push_back(it->second.point);
		for (int k = 0; k < it->second.track.size(); k++) {
			if (it->second.track[k] > 0) {
				matchesUsedFor3d[k].push_back(
						graph.obsValues[k][it->second.track[k]]);
				visibility[k].push_back(1);
			} else {
				matchesUsedFor3d[k].push_back(Point2d(-1, -1));
				visibility[k].push_back(0);
			}
		}
	}
	//start of bundle adjustment
	cvsba::Sba::Params params;
	params.type = cvsba::Sba::MOTIONSTRUCTURE;
	params.iterations = 200;
	params.minError = 1e-10;
	params.fixedIntrinsics = 5;
	params.fixedDistortion = 5;
	params.verbose = false;
	cvsba::Sba sba;
	sba.setParams(params);
	sba.run(points_opt, matchesUsedFor3d, visibility, cameraMatrix, R_opt, T_opt,
			distCoeffs);
	//bundleAdjust(points_opt, matchesUsedFor3d, visibility, cameraMatrix, R_opt,
	//		T_opt, distCoeffs, criteria, 0, 0);
	graph.pointCloud = points_opt;
	it = graph.pointMap.begin();
	graph.relativePoses.clear();
	for (int k = 0; k < R_opt.size(); k++) {
		Matx34d pose;
		createPoseFromRotationTranslation(R_opt[k], T_opt[k], pose);
		graph.relativePoses.push_back(pose);
	}
}
