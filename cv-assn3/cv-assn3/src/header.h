#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <vector>
#include <type_traits>
#include <cmath>
//===================================================================
using cv::Mat;
using cv::Scalar;
using cv::Point2f;
using cv::circle;
using cv::imread;
using cv::imshow;
using cv::waitKey;
using std::cout;
using std::vector;
//===================================================================
constexpr size_t N = 28;
//===================================================================
template <class T, size_t dim1, size_t dim2>
Mat copy_2_mat(const T(&arr)[dim1][dim2])
{
	Mat mat(dim1, dim2, CV_64FC1);
	for (int i = 0; i < dim1; ++i)
		for (int j = 0; j < dim2; ++j)
			mat.at<double>(i, j) = arr[i][j];
	return mat;
}
//===================================================================
Mat to_homo(const Mat& x)
{
	// Map a vector in non-homogeneous coordinates (dimension m)
	//  to a vector in homogeneous coordinates     (dimension m+1)

	Mat x_ = Mat::ones(x.rows + 1, x.cols, CV_64FC1);
	for (int i = 0; i < x.rows; ++i)
		for (int j = 0; j < x.cols; ++j)
			x_.at<double>(i, j) = x.at<double>(i, j);

	return x_;
}
//===================================================================
Mat from_homo(const Mat& x_) // NEED TO TEST THAT THIS WORKS AS EXPECTED!!!!!!!!!!!!!!!!!!
{
	// Map a vector in homogeneous coordinates     (dimension m+1)
	//  to a vector in non-homogeneous coordinates (dimension m)

	const size_t rows = x_.rows;
	const size_t cols = x_.cols;

	Mat x = Mat::ones(rows - 1, cols, CV_64FC1);
	for (int i = 0; i < rows - 1; ++i)
		for (int j = 0; j < cols; ++j)
			x.at<double>(i, j) = x_.at<double>(i, j) / x_.at<double>(rows-1, j);

	return x;
}
//===================================================================
Mat construct(const Mat& x_j, const Mat& X_j)
{
	// To match H&Z notation
	auto x = x_j.at<double>(0, 0);
	auto y = x_j.at<double>(0, 1);
	auto w = x_j.at<double>(0, 2);

	// DEBUG:
	//cout << "x = " << x << "\n";
	//cout << "y = " << y << "\n";
	//cout << "w = " << w << "\n";
	//cout << "x_j: \n" << x_j;
	//cout << "X_j: \n" << X_j;

	// Construct row-1
	Mat temp1 = Mat::zeros(1, 4, CV_64FC1);
	Mat temp2 = -w * X_j;
	Mat temp3 =  y * X_j;

	Mat row1;
	hconcat(temp1, temp2, row1);
	hconcat(row1, temp3, row1);
	//cout << "\nrow1:\n" << row1;

	// Construct row-2
	temp1 = w * X_j;
	temp2 = Mat::zeros(1, 4, CV_64FC1);
	temp3 = -x * X_j;

	Mat row2;
	hconcat(temp1, temp2, row2);
	hconcat(row2, temp3, row2);
	//cout << "\nrow2:\n" << row2;

	// Concatenate two rows rows
	Mat A_two_rows;
	vconcat(row1, row2, A_two_rows);

	return A_two_rows;
}
//===================================================================
Mat construct_A(const Mat& x_, const Mat& X_)
{
	// Return 3x12 sub - matrix for A corresponding to one 2D<->3D correspondance

	// This function places the points in the matrix seen in equation 7.2 (pg178)
	// with notation described also on page p89 in equation 4.1
	//   in Hartley and Zisserman

	// x is one 2D - point in the plane stored as col - vector
	// X is one 3D - point in the world stored as col - vector

	assert(x_.cols == X_.cols);
	const size_t N = x_.cols;
	//Mat A = Mat::zeros(2 * N, 12, CV_64FC1);

	Mat A;// = Mat::zeros(0, 0, CV_64FC1);


	// Itterate over correspondances to produce two rows of A
	for (int j = 0; j < N; ++j) // Itterate across cols ->
	{
		// Copy over one col by itterating down rows of current col
		Mat x_j(1, 3, CV_64FC1);
		for (int i = 0; i < 3; ++i) // 3D-homo-camera plane points
			x_j.at<double>(0, i) = x_.at<double>(i, j); // Copy col-vector into row-vector
	
		Mat X_j(1, 4, CV_64FC1); // Copy col-vector into row-vector
		for (int i = 0; i < 4; ++i) // 4D-homo-world points
			X_j.at<double>(0, i) = X_.at<double>(i, j); // Copy col-vector into row-vector

		if (j == 0)
			A = construct(x_j, X_j);
		else
		{
			auto temp = construct(x_j, X_j);
			vconcat(A, temp, A);
		}
	}


	return A;
}
//===================================================================
void draw_points(
	Mat& img_c, 
	vector<double> x_u, 
	vector<double> x_v,
	Scalar color = Scalar(255, 0, 0))
{
	// Place points in form to be drawn
	Point2f point1(x_u[0], x_v[0]);
	vector<Point2f> pts1({ point1 });
	for (int j = 1; j < N; ++j)
		pts1.push_back(Point2f(x_u[j], x_v[j]));

	// Draw points on original image
	int radius = 2;
	//Scalar color(255, 0, 0);
	vector<Scalar> colors{ Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0, 255), Scalar(255, 255, 0) };
	int thickness;

	// Look at mapping of 3-points for affine transformation
	int i = 0;
	for (auto itter : pts1)
		circle(img_c, itter, radius, color, thickness = 4);
		//cv::circle(img_c, itter, radius, colors[i++ % 3], thickness = 8);


}
//===================================================================
struct Proj_struct
{
	Mat P;
	Mat K;
	Mat R;
	Mat t;

	void P_to_KRt()//const Mat& P)
	{
		//	This function computes the decomposition of the projection matrix into intrinsic parameters, K, and extrinsic parameters Q(the rotation matrix) and t(the translation vector)
		//
		//  Usage:
		//	K, Q, t = P_to_KRt(P)
		//
		//		Input :
		//		P : 3x4 projection matrix
		//
		//		Outputs :
		//  K: 3x3 camera intrinsics
		//  Q : 3x3 rotation matrix(extrinsics)
		//  t : 3x1 translation vector(extrinsics)

		int x = 0, y = 0;
		int delta = 3;
		cv::Mat M = cv::Mat(P, cv::Rect(x, y, delta, delta));

		Mat Q;
		RQDecomp3x3(M, R, Q);

		//K = R/float(R[2,2])
		K = R / R.at<double>(2, 2);

		//if K[0, 0] < 0:
		//  K[:, 0] = -1 * K[:, 0]
		//	Q[0, :] = -1 * Q[0, :]
		if (K.at<double>(0, 0) < 0)
		{
			for (int i = 0; i < K.rows; ++i) // Itterate down rows of one column
				K.at<double>(i, 0) = -1 * K.at<double>(i, 0);

			for (int i = 0; i < Q.rows; ++i) // Itterate down rows of one column
				Q.at<double>(i, 0) = -1 * Q.at<double>(i, 0);
		}

		//if K[1, 1] < 0:
		//	K[:, 1] = -1 * K[:, 1]
		//	Q[1, :] = -1 * Q[1, :]
		if (K.at<double>(1, 1) < 0)
		{
			for (int i = 0; i < K.rows; ++i) // Itterate down rows of one column
				K.at<double>(i, 1) = -1 * K.at<double>(i, 1);

			for (int i = 0; i < Q.rows; ++i) // Itterate down rows of one column
				Q.at<double>(i, 1) = -1 * Q.at<double>(i, 1);
		}

		//if det(Q) < 0:
		//	print 'Warning: Determinant of the supposed rotation matrix is -1'
		if (determinant(Q) < 0)
			cout << "\nWarning: Determinant of the supposed rotation matrix is -1\n";;

		// P_3_3 = np.dot(K, Q)
		Mat P_3_3 = K * Q;

		// P_proper_scale = (P_3_3[0,0]*P)/float(P[0,0])
		Mat P_proper_scale = (P_3_3.at<double>(0, 0) * P) / float(P.at<double>(0, 0));

		//t = np.dot(inv(K), P_proper_scale[:, 3])
		Mat inv_K = K.inv();
		Mat P_proper_scale_slice(3, 1, CV_64FC1);
		for (int i = 0; i < P_proper_scale.rows; ++i)
			P_proper_scale_slice.at<double>(i, 0) = P_proper_scale.at<double>(i, 3); // Grab the third col 

		t = inv_K * P_proper_scale_slice;
	}
};
//===================================================================