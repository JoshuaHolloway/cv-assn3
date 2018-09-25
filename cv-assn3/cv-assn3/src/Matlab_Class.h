#pragma once
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include "engine.h"  // MATLAB Engine Header File required for building in Visual Studio 
#include "mex.h"
#include <windows.h>

using std::cout;
using std::string;
using cv::Mat;


class matlabClass
{
private:
	Engine * ep;				// Pointer to a MATLAB Engine
	mxArray *mx_Arr;  // To store the image data inside MATLAB
public:
	matlabClass() // Default constructor
	{


		// Start the MATLAB engine
		ep = engOpen(NULL);
		if (!(ep = engOpen("\0")))
		{
			// If matlab engine doesn't open then try to re-register 
			//	the matlab COM and try to open it again
			command("!matlab - regserver"); // Need to run as admin
			ep = engOpen(NULL);
			if (!(ep = engOpen("\0")))
			{
				std::cerr << "failed to start matlab" << std::endl;
				getchar();
				exit(1);
			}
		}

		// Open MATLAB GUI:
		command("desktop");

		// Reset MATLAB Environment
		command("clc, clear, close all;"); //engEvalString(ep, "clc, clear, close all;");

																			 // Change to active directory 
		string current_path = "cd " + ExePath();
		command(current_path); // Move to current directory of generated .exe	
		command("cd ../../../../../../..");  // Move to location of .m files
	}
	~matlabClass() // Destructor
	{
		//free(ep);
		engEvalString(ep, "close all;");
	}
	void command(string str)
	{
		auto error_flag = engEvalString(ep, str.c_str());
		//if (error_flag)
		//	command("lasterror");
	}

	template <typename T>
	void linearize(const Mat& mat_in, T* arr_out, const size_t M, const size_t N)
	{
		// Assume T is double
		Mat mat_local;
		mat_in.convertTo(mat_local, CV_64FC1);


		// mat_in needs to be CV_64FC1 if arr_out is double
		opencv_type(mat_in);


		//string ty = type2str(mat_in.type());
		//printf("Matrix: %s %dx%d \n", ty.c_str(), mat_in.cols, mat_in.rows);

		// Swap from row-major to col-major
		for (int row = 0; row < M; row++)
			for (int col = 0; col < N; col++)
				arr_out[col * M + row] = mat_local.at<T>(row, col);
	}

	template <typename T>
	T* transposeLin(const T* arrIn, const int M, const int N)
	{
		T* arrOut = (T*)malloc(sizeof(T) * M * N);
		for (int i = 0; i < M; ++i)
			for (int j = 0; j < N; ++j)
				arrOut[i*N + j] = arrIn[j*N + i];
		return arrOut;
	}

	// 2D cv::Mat passed from C++ into MATLAB
	void passImageIntoMatlab(const cv::Mat& img, string name)
	{
		// Convert the Mat object into a double array
		//double* linImgArrDouble = (double*)malloc(sizeof(double) * img.rows * img.cols);
		double* linImgArrDouble = new double[img.rows * img.cols];
		linearize(img, linImgArrDouble, img.rows, img.cols);

		// Copy image data into an mxArray inside C++ environment
		mx_Arr = mxCreateDoubleMatrix(img.rows, img.cols, mxREAL);
		memcpy(mxGetPr(mx_Arr), linImgArrDouble, img.rows * img.cols * sizeof(double));

		/// C++ -> MATLAB
		// Put variable into MATLAB workstpace
		engPutVariable(ep, name.c_str(), mx_Arr);
		delete[] linImgArrDouble;
	}

	// 2D data stored in 1D array passed into 2D matrix in MATLAB
	void pass_2D_into_matlab(const double* data, const int M, const int N, string name)
	{
		// Copy image data into an mxArray inside C++ environment
		mx_Arr = mxCreateDoubleMatrix(M, N, mxREAL);
		memcpy(mxGetPr(mx_Arr), data, M * N * sizeof(double));

		/// C++ -> MATLAB
		// Put variable into MATLAB workstpace
		engPutVariable(ep, name.c_str(), mx_Arr);
	}

	// 3D data stored in 1D array passed into 3D matrix in MATLAB
	void pass_3D_into_matlab(const double* data, const int dim1, const int dim2, const int dim3)
	{
		//3rd dim is (i,:,:) in C++, yet (:,:,i) in MATLAB
		//mxCreateNumericArray(mwSize ndim, const mwSize *dims,
		//	mxClassID classid, mxComplexity ComplexFlag);
		const mwSize ndim = 3;
		const mwSize dims[ndim] = { dim2, dim3, dim1 };
		mx_Arr = mxCreateNumericArray(ndim, dims, mxDOUBLE_CLASS, mxREAL);

		// Copy tensor data into an mxArray inside C++ environment
		memcpy(mxGetPr(mx_Arr), data, dim1 * dim2 * dim3 * sizeof(double));

		/// C++ -> MATLAB
		// Put variable into MATLAB workstpace
		engPutVariable(ep, "data_from_cpp", mx_Arr);
	}

	void getAudioFromMatlab()
	{
		// Read in audio file and play sound:
		engEvalString(ep, "[y,Fs] = audioread('handel.wav');");
		engEvalString(ep, "sound(y,Fs);");
	}

	double return_scalar_from_matlab(string variable)
	{
		// Grab value from workspace
		mxArray *cppValmxArray = engGetVariable(ep, variable.c_str());					// Pointer to MATLAB variable 
		double* cppValDblPtr = static_cast<double*>(mxGetData(cppValmxArray));	// Pointer to C variable
		std::cout << variable << " = " << *cppValDblPtr << std::endl << std::endl;
		return *cppValDblPtr;
	}

	void returnVectorFromMatlab(string variable)
	{
		mxArray *cppValmxArray = engGetVariable(ep, variable.c_str());								// Pointer to MATLAB variable 
		const double* cppValDblPtr = static_cast<double*>(mxGetData(cppValmxArray));	// Pointer to C variable
		std::cout << "Vector passed from MATLAB into C++ = " << cppValDblPtr[0] << " " << cppValDblPtr[1] << std::endl << std::endl;
	}

	cv::Mat return_matrix_as_cvMat_from_matlab(string variable)
	{

		mxArray* cppValmxArray = engGetVariable(ep, variable.c_str());															// Pointer to MATLAB variable 
		double* cppValDblPtr = static_cast<double*>(mxGetData(cppValmxArray));	// Pointer to C variable

																																						// Get dimensions from MATLAB
		double rows = 28;// return_scalar_from_matlab("rows");
		double cols = 28;// return_scalar_from_matlab("cols");

		cppValDblPtr = transposeLin(cppValDblPtr, (int)cols, (int)rows);


		//std::cout << "Vector passed from MATLAB into C++:\n";
		//for (int i = 0; i != (int)rows; ++i)
		//{
		//	for (int j = 0; j != (int)cols; ++j)
		//	{
		//		//cout << "(i,j) = " << "(" << i << ", " << j << ")    =>    " << cppValDblPtr[i * (int)cols + j] << "\n";
		//		cout << cppValDblPtr[i * (int)cols + j] << " ";
		//	}
		//	cout << "\n";
		//}

		//display_image(cppValDblPtr, cols, rows, false);

		return cv::Mat{ (int)rows, (int)cols, CV_64FC1, cppValDblPtr };
	}

	//template <typename T>
	//void fm_2_matlab_tensor(FeatureMap<T> fm)
	//{
	//	// Transpose each channels matrix and send to matlab to store from (i,:,:) -> (:,:,i)
	//	pass_3D_into_matlab(&fm.transpose()[0], fm.channels, fm.rows, fm.cols);

	//	// Execute the testbench script
	//	command("josh()");

	//	// Compute frobenius norm between MATLAB and C++
	//	return_scalar_from_matlab("error");
	//}


	// 4D data stored in 1D array passed into 3D matrix in MATLAB
	// NOTE: this will always be a 3D feature map stored as a s 4D tensor
	void pass_4D_into_matlab(const double* data, const int dim1, const int dim2, const int dim3, const int dim4)
	{
		//3rd dim is (i,:,:) in C++, yet (:,:,i) in MATLAB
		//           (2,3,4)             (3,4,2)
		//mxCreateNumericArray(mwSize ndim, const mwSize *dims,
		//	mxClassID classid, mxComplexity ComplexFlag);
		const mwSize ndim = 3;
		const mwSize dims[ndim] = { dim3, dim4, dim2 }; // Ignore the 1st dimension (see note above)
		mx_Arr = mxCreateNumericArray(ndim, dims, mxDOUBLE_CLASS, mxREAL);

		// Copy tensor data into an mxArray inside C++ environment
		memcpy(mxGetPr(mx_Arr), data, dim1 * dim2 * dim3 * dim4 * sizeof(double));

		/// C++ -> MATLAB
		// Put variable into MATLAB workstpace
		engPutVariable(ep, "data_from_cpp", mx_Arr);
	}

	//template <typename T>
	//void fm_2_matlab_vector(FeatureMap<T> vect)
	//{
	//	// Transpose each channels matrix and send to matlab to store from (i,:,:) -> (:,:,i)
	//	pass_2D_into_matlab(&vect[0], vect.rows, vect.cols); // column-vector => cols=1

	//	// Execute the testbench script
	//	command("josh()");

	//	// Compute frobenius norm between MATLAB and C++
	//	return_scalar_from_matlab("error");
	//}
	//=============================================================================
	void opencv_type(const cv::Mat& mat)
	{
		auto type = mat.type();
		auto depth = type & CV_MAT_DEPTH_MASK;
		auto chans = (type >> CV_CN_SHIFT) + 1;
		cout << "mat.type() = " << type << "\n";
		cout << "depth = " << depth << "\n";
		cout << "chans = " << chans << "\n";
		cout << "CV_8U = " << CV_8U << "\n";
		cout << "CV_8S = " << CV_8S << "\n";
		cout << "CV_16U = " << CV_16U << "\n";
		cout << "CV_16S = " << CV_16S << "\n";
		cout << "CV_32S = " << CV_32S << "\n";
		cout << "CV_32F = " << CV_32F << "\n";
		cout << "CV_64F = " << CV_64F << "\n";
	}
	//=============================================================================
	string ExePath() {
		char buffer[MAX_PATH];
		GetModuleFileName(NULL, buffer, MAX_PATH);
		const auto pos = string(buffer).find_last_of("\\/");
		return string(buffer).substr(0, pos);
	}
};
//=============================================================================