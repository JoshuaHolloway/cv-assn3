#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//===================================================================
using cv::Mat;
using cv::imread;
using cv::imshow;
using cv::waitKey;
//===================================================================
int main()
{
	auto img = imread("rubik_cube.jpg");
	imshow("test", img);
	waitKey(0);

	getchar();
	return 0;
}