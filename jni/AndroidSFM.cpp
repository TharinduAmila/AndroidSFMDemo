#include <headers/SFMlib.h>

using namespace cv;
using namespace std;

//holders of general data

vector<Mat> imageList;
vector<MFramePair> pairList;
vector<Point3d> denseCloud, sparseCloud;
MGraph outGraph;
Mat K(
		(Mat_<double>(3, 3) << 810.0, 0.0, 360.0, 0.0, 810.0, 240.0, 0.0, 0.0, 1.0));
Mat D(
		(Mat_<double>(5, 1) << 0.1908486258477136, -1.548531974562851, 0, 0, 3.008872628798979));
//Java wrapper to call Native methods implemented
extern "C" {
JNIEXPORT jint JNICALL
Java_com_example_androidsfmdemo_MainActivity_getImageCount(JNIEnv *env,
		jobject obj) {
	return imageList.size();
}
JNIEXPORT void JNICALL
Java_com_example_androidsfmdemo_MainActivity_viewDataFromNative(JNIEnv *env,
		jobject obj, jlong addrOut) {
	Mat& mRgb = *(Mat*) addrOut;
	string print;
	if (!imageList.empty())
		print = "Number of Images Captured = "
				+ convertIntToString(imageList.size());
	else
		print = "Press Capture";
	putTextOnTopOfMat(mRgb, print, 1, 2);
}
JNIEXPORT void JNICALL
Java_com_example_androidsfmdemo_MainActivity_sendImageToNative(JNIEnv *env,
		jobject obj, jlong addrOut) {
	Mat& mRgb = *(Mat*) addrOut;
	imageList.push_back(mRgb.clone());
	Mat out;
	//undistort(mRgb, out, K, D);
	//imageList.push_back(out);
}
JNIEXPORT void JNICALL
Java_com_example_androidsfmdemo_MainActivity_clearAllData(JNIEnv *env,
		jobject obj) {
	imageList.clear();
	pairList.clear();
	denseCloud.clear();
	sparseCloud.clear();
	outGraph.obsIndexes.clear();
	outGraph.obsValues.clear();
	outGraph.pointCloud.clear();
	outGraph.pointMap.clear();
	outGraph.relativePoses.clear();
}
JNIEXPORT jint JNICALL
Java_com_example_androidsfmdemo_MainActivity_runSparseReconstruction(
		JNIEnv *env, jobject obj) {
	for (int i = 0; i < imageList.size() - 1; i++) {
		MFramePair pair;
		pair.img1 = imageList[i];
		pair.img2 = imageList[i + 1];
		pairList.push_back(pair);
	}
	int sucsess = doSparseReconstruction(pairList, K, outGraph);
	if (sucsess == 1) {
		sparseCloud = outGraph.pointCloud;

	}
	return sucsess;
}
JNIEXPORT jdoubleArray JNICALL
Java_com_example_androidsfmdemo_Points_getSparsePointCloud(JNIEnv *env,
		jobject obj) {
	jdoubleArray result;
	int size = sparseCloud.size() * 3;
	result = env->NewDoubleArray(size);
	if (result == NULL) {
		return NULL; /* out of memory error thrown */

	}
	vector<double> X, Y, Z;
	for (int i = 0; i < sparseCloud.size(); i++) {
		X.push_back(sparseCloud[i].x);
		Y.push_back(sparseCloud[i].y);
		Z.push_back(sparseCloud[i].z);
	}
	std::nth_element(X.begin(), X.begin() + X.size() / 2, X.end());
	std::nth_element(Y.begin(), Y.begin() + Y.size() / 2, Y.end());
	std::nth_element(Z.begin(), Z.begin() + Z.size() / 2, Z.end());
	double centerX = X[X.size() / 2];
	double centerY = Y[Y.size() / 2];
	double centerZ = Z[Z.size() / 2];
	jdouble fill[size];
	for (int i = 0; i < size; i++) {
		int k = i / 3;
		if (i % 3 == 0)
			fill[i] = sparseCloud[k].x - centerX;
		else if (i % 3 == 1)
			fill[i] = sparseCloud[k].y - centerY;
		else if (i % 3 == 2)
			fill[i] = sparseCloud[k].z - centerZ;
	}
	// move from the temp structure to the java structure
	env->SetDoubleArrayRegion(result, 0, size, fill);
	return result;
}
JNIEXPORT jfloatArray JNICALL
Java_com_example_androidsfmdemo_Points_getSparseColorCloud(JNIEnv *env,
		jobject obj) {
	jfloatArray result;
	int size = outGraph.pointCloud.size() * 4;
	result = env->NewFloatArray(size);
	if (result == NULL) {
		return NULL; /* out of memory error thrown */

	}
	jfloat fill[size];
	std::map<int, worldData>::iterator it = outGraph.pointMap.begin();
	for (int i = 0; it != outGraph.pointMap.end(); it++, i++) {
		int index = i * 4;
		int imageIndex = 0;
		while (imageIndex < it->second.track.size()
				&& it->second.track[imageIndex] == -1) {
			imageIndex++;
		}
		if (imageIndex < it->second.track.size()) {
			Mat image;
			cvtColor(imageList[imageIndex], image, CV_RGBA2RGB);
			int k = it->second.track[imageIndex];
			Point2d point = outGraph.obsValues[imageIndex][k];
			Vec3b color = image.at<Vec3b>(point);
			fill[index] = color.val[0] / 255.f;
			fill[index + 1] = color.val[1] / 255.f;
			fill[index + 2] = color.val[2] / 255.f;
			fill[index + 3] = 1.0f;
		} else {
			fill[index] = 255 / 255.f;
			fill[index + 1] = 255 / 255.f;
			fill[index + 2] = 255 / 255.f;
			fill[index + 3] = 1.0f;
		}
	}
	// move from the temp structure to the java structure
	env->SetFloatArrayRegion(result, 0, size, fill);
	return result;
}
}

