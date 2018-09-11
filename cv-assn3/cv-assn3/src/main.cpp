#include "header.h"
#include "Matlab_Class.h"
#include <numeric>
int main()
{
	/// Part 1:
	// 2D points (data from: rubik_2D_pts.mat)
	const double pts_2d[2][N] = 
	{ 
		{ 553.174856674857,	742.291973791974,	936.021703521704,	1128.07411957412,	561.980753480754,	749.839885339885,	938.537674037674,	1121.36486486487,	573.721949221949,	754.871826371827,	941.053644553645,	1121.78419328419,	582.947174447175,	759.065110565111,	942.311629811630,	1121.78419328419,	562.819410319410,	744.807944307945,	931.828419328420,	1115.91359541360,	576.657248157248,	750.259213759214,	928.893120393121,	1103.75307125307,	587.559787059787,	754.452497952498,	926.377149877150,	1095.36650286650 },
		{ 583.156838656839,	586.930794430794,	593.640049140049,	598.671990171990,	429.682637182637,	430.521294021294,	432.198607698608,	436.391891891892,	270.337837837838,	273.692465192465,	276.627764127764, 279.143734643735,	126.927518427519,	127.346846846847,	131.540131040131,	133.636773136773,	680.441031941032,	681.699017199017,	685.472972972973,	688.827600327601,	766.403357903358,	767.242014742015,	769.757985257986,	773.531941031941,	843.140458640459,	841.463144963145,	843.979115479116,	850.269041769042 }
	};
	
	// 3D points (data from: rubik_3D_pts.mat)
	const double pts_3d[3][N] =
	{
		{ 0,	1,	2,	3,	0,	1,	2,	3,	0,	1,	2,	3,	0,	1,	2,	3,	0,	1,	2,	3,	0,	1,	2,	3,	0,	1,	2,	3 },
		{ 0,	0,	0,	0,	1,	1,	1,	1,	2,	2,	2,	2,	3,	3,	3,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0 },
		{ 0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0, -1, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3, -3 }
	};

	auto x_1 = copy_2_mat(pts_2d);
	auto X_1 = copy_2_mat(pts_3d);

	auto x_1_ = to_homo(x_1);
	auto X_1_ = to_homo(X_1);
	
	// Construct the matrix A corresponding to equation 7.2 (page 178 in H&Z)
	auto A = construct_A(x_1_, X_1_);

	// Drop an SVD on A
	Mat W, U, Vt;
	cv::SVD::compute(A, W, U, Vt);
	
	// Modify SVD
	Mat V = -Vt.t(); // SVD in OpenCV is different than MATLAB and Numpy

	// Extract right most column of V
	const size_t I = V.rows;
	const size_t J = V.cols;
	Mat v(I, 1, CV_64FC1); // Col-vector to store right most col of V
	for (int i = 0; i < I; ++i)
		v.at<double>(i, 0) = V.at<double>(i, J-1); // Itterate down right-most col of V

	// Re-shape into 3x4 camera-projection matrix
	int cn=0;
	int rows=3;
	Mat P = v.reshape(cn,rows);
	//cout << "P:\n" << P;

	// Re-project the 3D-world points into the 2D-image plane
	auto x_1_reproject_ = P * X_1_;
	//cout << "x_1_reproject" << x_1_reproject_;

	// Map from homo to non-homo coordinates:
	auto x_1_reproject = from_homo(x_1_reproject_);
	cout << "x_1_reproject" << x_1_reproject;

	
	/// Compute re-projection error

	// Get into form that matches notation from LM-implementation notes:
	vector<double> x_u,     x_v;      // row-vec
	vector<double> x_u_hat, x_v_hat;  // row-vec
	for (int j = 0; j < N; ++j) // itterate across cols
	{
		x_u.push_back(x_1.at<double>(0, j));
		x_v.push_back(x_1.at<double>(1, j));

		x_u_hat.push_back(x_1_reproject.at<double>(0, j));
		x_v_hat.push_back(x_1_reproject.at<double>(1, j));
	}
	
	// Form d-vector
	vector<double> d;
	double l2 = 0;
	for (int j = 0; j < N; ++j)
	{
		auto d_jx = x_u[j] - x_u_hat[j];
		auto d_jy = x_v[j] - x_v_hat[j];

		d.push_back(d_jx);
		d.push_back(d_jy);

		auto diff = d_jx - d_jy;
		l2 += diff * diff;
	}

	// Perform inner product with self
	auto sum_sqaured_error = std::inner_product(d.begin(), d.end(), d.begin(), 0);
	cout << "sum_sqaured_error = " << sum_sqaured_error << "\n";
	cout << "l2 error = " << l2 << "\n";

	// Look at image
	auto img = imread("rubik_cube.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	Mat img_c1; 
	cvtColor(img, img_c1, cv::COLOR_GRAY2BGR);

	// Place poitns in form to be drawn
	const int x1 = 50, y1 = 50;   // (x1,y1) ----- (x2,y1)
	const int x2 = 200, y2 = 200; //    |             |
	const int delta = 50;         //    |             |
                                // (x1,y2) ----- (x2,y2)
	Point2f point1(x1, y1), point2(x2, y1);
	Point2f point3(x1, y2), point4(x2, y2);	vector<Point2f> pts1({ point1, point2, point3 });


	// Draw points on original image
	int radius = 2;
	Scalar color(255, 0, 0);
	vector<Scalar> colors{ Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0, 255), Scalar(255, 255, 0) };
	int thickness;
	
	// Look at mapping of 3-points for affine transformation
	int i = 0;
	for (auto itter : pts1)
		cv::circle(img_c1, itter, radius, colors[i++], thickness = 8);

	imshow("Original with 3-points", img_c1);
	waitKey(0);

	/// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	/// Protoyping stuff:
	/// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	// Link to MATLAB environment
	matlabClass matlab;
	matlab.passImageIntoMatlab(x_1_reproject);
	//matlab.passImageIntoMatlab(A);

	// Run MATLAB script that executes prototype
	matlab.command("ass3");

	
	return 0;
}
