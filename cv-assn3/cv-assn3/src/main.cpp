#include "header.h"
#include "Matlab_Class.h"
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

	//cout << "A:\n" << A;



	// TODO - drop SVD on A
	// [U, SIGMA, V] = svd(A);

	Mat W, U, Vt;
	cv::SVD::compute(A, W, U, Vt);
	//cout << "Vt:\n" << vt;

	Mat V = -Vt.t(); // SVD is only unique up to a sign
	//cout << "V:\n" << V;

	// Extract right most column of V
	const size_t I = V.rows;
	const size_t J = V.cols;
	Mat v(I, 1, CV_64FC1); // Col-vector to store right most col of V
	for (int i = 0; i < I; ++i)
		v.at<double>(i, 0) = V.at<double>(i, J-1); // Itterate down right-most col of V

	//cout << "v:\n" << v;
	int cn=0;
	int rows=3;
	Mat P = v.reshape(cn,rows);
	cout << "P:\n" << P;


	// Link to MATLAB environment
	matlabClass matlab;
	matlab.passImageIntoMatlab(P);
	//matlab.passImageIntoMatlab(A);

	// Run MATLAB script that executes prototype
	matlab.command("ass3");

	// TODO - extract right-most column of V


	auto img = imread("rubik_cube.jpg");
	imshow("test", img);
	waitKey(0);

	getchar();
	return 0;
}
