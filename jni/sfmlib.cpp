#include <jni.h>
#include <opencv2/opencv.hpp>

#define EPSILON 0.0001
using namespace std;
using namespace cv;

ORB detector(1000);
Mat image;
vector<Mat> imageList;
Mat_<double> E;
Mat K(3, 3, CV_64F);
vector<vector<vector<Point2f> > > matchesBetweenImages;
vector<vector<vector<KeyPoint> > > matchesUsedFor3d;
Matx34d P;
Matx34d P1;
vector<Point3d> pointCloud;
bool ratio = false;
double reError = 0;
int matcherAlgo = 0;

void KeyPointsToPoints(vector<KeyPoint> in, vector<Point2f>& out) {
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

bool CheckCoherentRotation(cv::Mat_<double>& R) {
	if (fabsf(determinant(R)) - 1.0 > 1e-07) {
		return false;
	}
	return true;
}
Mat_<double> LinearLSTriangulation(Point3d u,	//homogenous image point (u,v,1)
		Point3d u1, 	//homogenous image point in 2nd camera)
		Matx34d P, Matx34d P1) { 	//camera matrices
	//build A matrix
	Matx43d A(u.x * P(2, 0) - P(0, 0), u.x * P(2, 1) - P(0, 1),
			u.x * P(2, 2) - P(0, 2), u.y * P(2, 0) - P(1, 0),
			u.y * P(2, 1) - P(1, 1), u.y * P(2, 2) - P(1, 2),
			u1.x * P1(2, 0) - P1(0, 0), u1.x * P1(2, 1) - P1(0, 1),
			u1.x * P1(2, 2) - P1(0, 2), u1.y * P1(2, 0) - P1(1, 0),
			u1.y * P1(2, 1) - P1(1, 1), u1.y * P1(2, 2) - P1(1, 2));
//build B vector
	Matx41d B(-(u.x * P(2, 3) - P(0, 3)), -(u.y * P(2, 3) - P(1, 3)),
			-(u1.x * P1(2, 3) - P1(0, 3)), -(u1.y * P1(2, 3) - P1(1, 3)));
//solve for X
	Mat_<double> X;
	solve(A, B, X, DECOMP_SVD);
	return X;
}

Mat_<double> IterativeLinearLSTriangulation(Point3d u, //homogenous image point (u,v,1)
		Matx34d P,          //camera 1 matrix
		Point3d u1,         //homogenous image point in 2nd camera
		Matx34d P1          //camera 2 matrix
		) {
	double wi = 1, wi1 = 1;
	Mat_<double> X(4, 1);
	for (int i = 0; i < 14; i++) { //Hartley suggests 10 iterations at most
		Mat_<double> X_ = LinearLSTriangulation(u, u1, P, P1);
		X(0) = X_(0);
		X(1) = X_(1);
		X(2) = X_(2);
		X_(3) = 1.0;

		//recalculate weights
		double p2x = Mat_<double>(Mat_<double>(P).row(2) * X)(0);
		double p2x1 = Mat_<double>(Mat_<double>(P1).row(2) * X)(0);

		//breaking point
		if (fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON)
			break;

		wi = p2x;
		wi1 = p2x1;

		//reweight equations and solve
		Matx43d A((u.x * P(2, 0) - P(0, 0)) / wi,
				(u.x * P(2, 1) - P(0, 1)) / wi, (u.x * P(2, 2) - P(0, 2)) / wi,
				(u.y * P(2, 0) - P(1, 0)) / wi, (u.y * P(2, 1) - P(1, 1)) / wi,
				(u.y * P(2, 2) - P(1, 2)) / wi,
				(u1.x * P1(2, 0) - P1(0, 0)) / wi1,
				(u1.x * P1(2, 1) - P1(0, 1)) / wi1,
				(u1.x * P1(2, 2) - P1(0, 2)) / wi1,
				(u1.y * P1(2, 0) - P1(1, 0)) / wi1,
				(u1.y * P1(2, 1) - P1(1, 1)) / wi1,
				(u1.y * P1(2, 2) - P1(1, 2)) / wi1);
		Mat_<double> B =
				(Mat_<double>(4, 1) << -(u.x * P(2, 3) - P(0, 3)) / wi, -(u.y
						* P(2, 3) - P(1, 3)) / wi, -(u1.x * P1(2, 3) - P1(0, 3))
						/ wi1, -(u1.y * P1(2, 3) - P1(1, 3)) / wi1);

		solve(A, B, X_, DECOMP_SVD);
		X(0) = X_(0);
		X(1) = X_(1);
		X(2) = X_(2);
		X_(3) = 1.0;
	}
	return X;
}
double TriangulatePoints(const vector<KeyPoint>& pt_set1,
		const vector<KeyPoint>& pt_set2, const Mat&Kinv, const Matx34d& P,
		const Matx34d& P1, vector<Point3d>& pointcloud) {
	vector<double> reproj_error;
	pointcloud.clear();
	for (unsigned int i = 0; i < pt_set1.size(); i++) {
//convert to normalized homogeneous coordinates
		Point2f kp = pt_set1[i].pt;
		Point3d u(kp.x, kp.y, 1.0);
		Mat_<double> um = Kinv * Mat_<double>(u);
		u = um.at<Point3d>(0);
		Point2f kp1 = pt_set2[i].pt;
		Point3d u1(kp1.x, kp1.y, 1.0);
		Mat_<double> um1 = Kinv * Mat_<double>(u1);
		u1 = um1.at<Point3d>(0);
//triangulate
		Mat_<double> X = IterativeLinearLSTriangulation(u, P, u1, P1); //LinearLSTriangulation(u, u1,P,P1);
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
void createPoseFromRotationTranslation(Mat_<double> R, Mat_<double> t,
		Matx34d& Pose) {
	Pose = Matx34d(R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0), R(1, 1), R(1, 2),
			t(1), R(2, 0), R(2, 1), R(2, 2), t(2));
}
bool checkCorrectnessOfPose(Matx34d & Pose, vector<KeyPoint> key1,
		vector<KeyPoint> key2, Mat_<double> Kinv) {
	vector<Point3d> pcloud_pt3d;
	TriangulatePoints(key1, key2, Kinv, P, Pose, pcloud_pt3d);
	pointCloud = pcloud_pt3d;
	vector<Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());

	Matx44d P4x4 = Matx44d::eye();
	for (int i = 0; i < 12; i++)
		P4x4.val[i] = Pose.val[i];

	perspectiveTransform(pcloud_pt3d, pcloud_pt3d_projected, P4x4);
	int count = 0;
	for (int i = 0; i < pcloud_pt3d.size(); i++) {
		count += (pcloud_pt3d_projected[i].z > 0) ? 1 : 0;
	}
	double percentage = ((double) count / (double) pcloud_pt3d.size());
	if (percentage < 0.75)
		return false; //less than 75% of the points are in front of the camera

	//check for coplanarity of points
	if (true) //not
	{
		cv::Mat_<double> cldm(pcloud_pt3d.size(), 3);
		for (unsigned int i = 0; i < pcloud_pt3d.size(); i++) {
			cldm.row(i)(0) = pcloud_pt3d[i].x;
			cldm.row(i)(1) = pcloud_pt3d[i].y;
			cldm.row(i)(2) = pcloud_pt3d[i].z;
		}
		cv::Mat_<double> mean;
		cv::PCA pca(cldm, mean, CV_PCA_DATA_AS_ROW);

		int num_inliers = 0;
		cv::Vec3d nrm = pca.eigenvectors.row(2);
		nrm = nrm / norm(nrm);
		cv::Vec3d x0 = pca.mean;
		double p_to_plane_thresh = sqrt(pca.eigenvalues.at<double>(2));

		for (int i = 0; i < pcloud_pt3d.size(); i++) {
			Vec3d w = Vec3d(pcloud_pt3d[i]) - x0;
			double D = fabs(nrm.dot(w));
			if (D < p_to_plane_thresh)
				num_inliers++;
		}
		if ((double) num_inliers / (double) (pcloud_pt3d.size()) > 0.85)
			return false;
	}
	return true;
}
void findCorrectPose(Mat svd_u, Mat svd_vt, Matx33d W, Matx34d& Pose,
		vector<KeyPoint> key1, vector<KeyPoint> key2, Mat_<double> Kinv) {
	Mat_<double> R = svd_u * Mat(W) * svd_vt; //HZ 9.19
	Mat_<double> R1 = svd_u * Mat(W).t() * svd_vt; //HZ 9.19
	Mat_<double> t = svd_u.col(2); // u3
	Mat_<double> tneg = -1 * svd_u.col(2); // -u3
	//step added to refine R
	SVD rSVD(R);
	R = rSVD.u * Mat::eye(3, 3, CV_64F) * rSVD.vt;
	double myscale = trace(rSVD.w)[0] / 3;
	t = t / myscale;
	SVD r1SVD(R1);
	R1 = r1SVD.u * Mat::eye(3, 3, CV_64F) * r1SVD.vt;
	double myscale1 = trace(r1SVD.w)[0] / 3;
	tneg = tneg / myscale1;
	if (!CheckCoherentRotation(R) || !CheckCoherentRotation(R1)) {
		Pose = 0;
		return;
	}
	//check to see what combination of R/R1 t/tneg gives the correct triangulation

	Matx34d TempPose;
	createPoseFromRotationTranslation(R, t, Pose);
	if (checkCorrectnessOfPose(Pose, key1, key2, K.inv())) {
		Pose = TempPose;
		return;
	}
	createPoseFromRotationTranslation(R, tneg, Pose);
	if (checkCorrectnessOfPose(Pose, key1, key2, K.inv())) {
		Pose = TempPose;
		return;
	}
	createPoseFromRotationTranslation(R1, t, Pose);
	if (checkCorrectnessOfPose(Pose, key1, key2, K.inv())) {
		Pose = TempPose;
		return;
	}
	createPoseFromRotationTranslation(R1, tneg, Pose);
	if (checkCorrectnessOfPose(Pose, key1, key2, K.inv())) {
		Pose = TempPose;
		return;
	}
}
int findEssentialMatrix(vector<Point2f> imgpts1, vector<Point2f> imgpts2) {
	vector<KeyPoint> k1, k2;
	K =
			(Mat_<double>(3, 3) << 800.0, 0.0, 360.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0);
	P = Matx34d(1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0);

	vector<uchar> status(imgpts1.size());
	Mat F = findFundamentalMat(imgpts1, imgpts2, CV_FM_LMEDS, 0.1, 0.99, status);
	E = K.t() * F * K; //according to HZ (9.12)
	//decompose E to P' , HZ (9.19)
	SVD svd(E, SVD::MODIFY_A);
	Mat svd_u = svd.u;
	Mat svd_vt = svd.vt;
	Mat svd_w = svd.w;
	Matx33d W(0, -1, 0,	//HZ 9.13
			1, 0, 0, 0, 0, 1);
	/////////////////////////////////////////////////////
	for (unsigned int i = 0; i < status.size(); i++) { // queryIdx is the "left" image
		if (status[i]) {
			k1.push_back(KeyPoint(imgpts1[i], CV_32F));
			k2.push_back(KeyPoint(imgpts2[i], CV_32F));
		}
	}
	findCorrectPose(svd_u, svd_vt, W, P1, k1, k2, K.inv());
	if (k1.size() > 0) {
		vector<vector<KeyPoint> > temp;
		temp.push_back(k1);
		temp.push_back(k2);
		matchesUsedFor3d.push_back(temp);
		return 1;
	} else
		return 0;
}

void OFmatch(Mat img1, Mat img2) {
// Detect keypoints in the left and right images
	// making sure images are grayscale
	Mat prevgray, gray;
	cvtColor(img1, prevgray, CV_RGBA2GRAY);
	cvtColor(img2, gray, CV_RGBA2GRAY);
	GaussianBlur(prevgray, prevgray, Size(5, 5), 0, 0);
	GaussianBlur(gray, gray, Size(5, 5), 0, 0);
	vector<KeyPoint> left_keypoints, right_keypoints;
	FastFeatureDetector ffd(8, true);
	ffd.detect(prevgray, left_keypoints);
	ffd.detect(gray, right_keypoints);
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
	vector<Point2f> imgpts1, imgpts2;
	for (unsigned int i = 0; i < matches.size(); i++) { // queryIdx is the "left" image
		imgpts1.push_back(left_keypoints[matches[i].queryIdx].pt);
// trainIdx is the "right" image
		imgpts2.push_back(right_keypoints[matches[i].trainIdx].pt);
	}
	//findEssentialMatrix(imgpts1, imgpts2);
	vector<vector<Point2f> > temp;
	temp.push_back(imgpts1);
	temp.push_back(imgpts2);
	matchesBetweenImages.push_back(temp);
}

void denseOFmatch(Mat img1, Mat img2, Mat& flow, int step) {
	Mat prevgray, gray;
	cvtColor(img1, prevgray, CV_RGBA2GRAY);
	cvtColor(img2, gray, CV_RGBA2GRAY);

	GaussianBlur(prevgray, prevgray, Size(5, 5), 0, 0);
	GaussianBlur(gray, gray, Size(5, 5), 0, 0);

	calcOpticalFlowFarneback(prevgray, gray, flow, 0.3, 8, 30, 10, 5, 1.2,
			OPTFLOW_FARNEBACK_GAUSSIAN);
	vector<Point2f> imgPnt1, imgPnt2;
	for (int y = 0; y < flow.rows; y += step)
		for (int x = 0; x < flow.cols; x += step) {
			const Point2f& fxy = flow.at<Point2f>(y, x);
			float mag = sqrt(fxy.x * fxy.x + fxy.y * fxy.y);
			if (mag > 5 && cvRound(x + fxy.x) > 0
					&& cvRound(x + fxy.x) < flow.cols && cvRound(y + fxy.y) > 0
					&& cvRound(y + fxy.y) < flow.rows) { //filter to use only points with a min movement
				imgPnt1.push_back(Point2f(x, y));
				imgPnt2.push_back(
						Point2f(cvRound(x + fxy.x), cvRound(y + fxy.y)));
			}
		}
	//findEssentialMatrix(imgPnt1, imgPnt2);
	vector<vector<Point2f> > temp;
	temp.push_back(imgPnt1);
	temp.push_back(imgPnt2);
	matchesBetweenImages.push_back(temp);
}

int getMatchesUsingORB(Mat img1, Mat img2) {
	Mat gray1, gray2, desc1, desc2;
	vector<KeyPoint> kpnt1, kpnt2;
	cvtColor(img1, gray1, CV_RGBA2GRAY);
	cvtColor(img2, gray2, CV_RGBA2GRAY);
	detector.detect(gray1, kpnt1);
	detector.detect(gray2, kpnt2);
	detector.compute(gray1, kpnt1, desc1);
	detector.compute(gray1, kpnt1, desc2);

	vector<DMatch> temp;
	vector<vector<DMatch> > kmatch;
	if (ratio) {
		BFMatcher matcher(cv::NORM_HAMMING, false);
		matcher.knnMatch(desc1, desc2, kmatch, 2);
		const float minRatio = 1.f / 1.5f;
		for (size_t i = 0; i < kmatch.size(); i++) {
			const cv::DMatch& bestMatch = kmatch[i][0];
			const cv::DMatch& betterMatch = kmatch[i][1];
			float distanceRatio = bestMatch.distance / betterMatch.distance;
			if (distanceRatio < minRatio) {
				temp.push_back(bestMatch);
			}
		}
	} else {
		BFMatcher matcher(cv::NORM_HAMMING, true);
		matcher.match(desc1, desc2, temp);
	}
	vector<Point2f> imgPnt1, imgPnt2;
	for (int i = 0; i < temp.size(); i++) {
		imgPnt1.push_back(kpnt1[temp[i].queryIdx].pt);
		imgPnt2.push_back(kpnt2[temp[i].trainIdx].pt);
	}
	//return findEssentialMatrix(imgPnt1, imgPnt2);
	vector<vector<Point2f> > tempory;
	tempory.push_back(imgPnt1);
	tempory.push_back(imgPnt2);
	matchesBetweenImages.push_back(tempory);
	return 0;
}

void mDrawMatches(Mat image1, Mat image2, Mat& out, vector<KeyPoint> k1,
		vector<KeyPoint> k2) {
	Mat i1, i2;
	cvtColor(image1, i1, CV_RGBA2RGB);
	cvtColor(image2, i2, CV_RGBA2RGB);
	Mat combine(image1.size().height, 2 * image2.size().width, CV_8UC3);
	Mat left_roi(combine,
			Rect(0, 0, image1.size().width, image1.size().height));
	i1.copyTo(left_roi);
	Mat right_roi(combine,
			Rect(image1.size().width, 0, image1.size().width,
					image1.size().height));
	i2.copyTo(right_roi);
	for (int i = 0; i < k1.size(); i++) {
		line(combine, k1[i].pt,
				Point2f(k2[i].pt.x + image1.size().width, k2[i].pt.y),
				Scalar(rand() % 255, rand() % 255, rand() % 255));
	}
	combine.copyTo(out);
}

extern "C" {
JNIEXPORT jint JNICALL
Java_com_example_sfm_MainActivity_match(JNIEnv *env, jobject obj,
		jlong addrOut) {
	Mat& mRgb = *(Mat*) addrOut;
	Mat image1 = imageList[0];
	Mat image2 = imageList[imageList.size() - 1];
	ratio = false;
	Mat out;
	int success = 1;
	if (matcherAlgo == 0) {
		//getMatchesUsingORB(image1, image2);
	} else if (matcherAlgo == 1) {
		OFmatch(image1, image2);
	} else if (matcherAlgo == 2) {
		Mat flow;
		//denseOFmatch(image1, image2, flow, 10);
	}
//resize(out,out,image1.size());
	vector<KeyPoint> k1, k2;
	PointsToKeyPoints(matchesBetweenImages[0][0], k1);
	PointsToKeyPoints(matchesBetweenImages[0][1], k2);
	mDrawMatches(image1, image2, out, k1, k2);
	out.copyTo(mRgb);
	return success;
}
JNIEXPORT double JNICALL
Java_com_example_sfm_MainActivity_updateCurrentImage(JNIEnv *env, jobject obj,
		jlong addrIn) {
	Mat& mRgb = *(Mat*) addrIn;
	mRgb.copyTo(image);
	return reError;
}
JNIEXPORT void JNICALL
Java_com_example_sfm_MainActivity_setImage(JNIEnv *env, jobject obj,
		jlong addrOut) {
	Mat& mRgb = *(Mat*) addrOut;
	imageList.push_back(image.clone());
	image.copyTo(mRgb);
}
JNIEXPORT void JNICALL
Java_com_example_sfm_MainActivity_setMatcher(JNIEnv *env, jobject obj,
		jint in) {
	matchesUsedFor3d.clear();
	matchesBetweenImages.clear();
	matcherAlgo = in;
}
JNIEXPORT jdoubleArray JNICALL Java_com_example_sfm_Points_getPointsArray(
		JNIEnv *env, jobject obj) {
	if (!matchesBetweenImages.empty()) {
		pointCloud.clear();
		Mat kinv = K.inv();
		vector<KeyPoint> k1, k2;
		findEssentialMatrix(matchesBetweenImages[0][0],matchesBetweenImages[0][1]);
	}
	int i, size;
	if (pointCloud.empty()) {
		pointCloud.push_back(Point3d(0.0, 0.0, 0.0));
		pointCloud.push_back(Point3d(-1.0, -1.0, -1.0));
		pointCloud.push_back(Point3d(1.0, -1.0, -1.0));
		pointCloud.push_back(Point3d(1.0, 1.0, -1.0));
		pointCloud.push_back(Point3d(-1.0, 1.0, -1.0));
		pointCloud.push_back(Point3d(-1.0, -1.0, 1.0));
		pointCloud.push_back(Point3d(1.0, -1.0, 1.0));
		pointCloud.push_back(Point3d(1.0, 1.0, 1.0));
		pointCloud.push_back(Point3d(-1.0, 1.0, 1.0));
	}
	size = pointCloud.size() * 3;
	// fill a temp structure to use to populate the java int array
	jdouble fill[size];
	for (i = 0; i < size; i++) {
		int k = i / 3;
		if (i % 3 == 0)
			fill[i] = pointCloud[k].x;
		else if (i % 3 == 1)
			fill[i] = pointCloud[k].y;
		else if (i % 3 == 2)
			fill[i] = pointCloud[k].z;
	}
	jdoubleArray result;
	result = env->NewDoubleArray(size);
	if (result == NULL) {
		return NULL; /* out of memory error thrown */
	}
	// move from the temp structure to the java structure
	env->SetDoubleArrayRegion(result, 0, size, fill);
	return result;
}

JNIEXPORT jfloatArray JNICALL Java_com_example_sfm_Points_getColorsArray(
		JNIEnv *env, jobject obj) {
	int size = 0;
	vector<KeyPoint> k1;
	if(!matchesUsedFor3d.empty()){k1 =matchesUsedFor3d[0][0];}
	if (k1.empty()) {
	jfloat fill[9 * 4];
	size = 9 * 4;
	for (int i = 0; i < 9; i++) {
		fill[i * 4] = ((rand() % 100 + 10) / 110.f);
		fill[i * 4 + 1] = ((rand() % 100 + 10) / 110.f);
		fill[i * 4 + 2] = ((rand() % 100 + 10) / 110.f);
		fill[i * 4 + 3] = 1.0f;
	}
	jfloatArray result;
	result = env->NewFloatArray(size);
	if (result == NULL) {
		return NULL; /* out of memory error thrown */
	}
	// move from the temp structure to the java structure
	env->SetFloatArrayRegion(result, 0, size, fill);
	return result;
	} else {
		size = k1.size() * 4;
		jfloat fill[size];
		Mat matC;
		cvtColor(imageList[0], matC, CV_RGBA2RGB);
		for (int i = 0; i < k1.size(); i++) {
			fill[i * 4] = (matC.at<Vec3b>(k1[i].pt)).val[0] / 255.f;
			fill[i * 4 + 1] = (matC.at<Vec3b>(k1[i].pt)).val[1] / 255.f;
			fill[i * 4 + 2] = (matC.at<Vec3b>(k1[i].pt)).val[2] / 255.f;
			fill[i * 4 + 3] = 1.0f;
		}
		jfloatArray result;
		result = env->NewFloatArray(size);
		if (result == NULL) {
			return NULL; /* out of memory error thrown */
		}
		// move from the temp structure to the java structure
		env->SetFloatArrayRegion(result, 0, size, fill);
		return result;
	}
	return NULL;
}

JNIEXPORT void JNICALL
Java_com_example_sfm_MainActivity_ClearAll(JNIEnv *env, jobject obj) {
	imageList.clear();
	matchesBetweenImages.clear();
	matchesUsedFor3d.clear();
	E = Mat();
	K = Mat(3, 3, CV_64F);
	P = Matx34d();
	P1 = Matx34d();
	pointCloud.clear();
}
}
