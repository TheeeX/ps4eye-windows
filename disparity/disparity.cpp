
#include "disparity.h"

#include <stdio.h>
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

const char *windowDisparity = "Disparity";


int disparity_map(Mat imgLeft, Mat imgRight, int ndisparities, int SADWindowSize)
{
	cvtColor(imgLeft, imgLeft, CV_BGR2GRAY);
	cvtColor(imgRight, imgRight, CV_BGR2GRAY);

	Mat test_img;
	int imn = 2;
	if (imn == 1)
	{
		test_img = imread("C:\\imlr1.jpg", IMREAD_GRAYSCALE);
		waitKey(33);
		imgLeft = test_img(Range(0, test_img.rows - 1), Range(0, test_img.cols / 2 - 1));
		imgRight = test_img(Range(0, test_img.rows - 1), Range(test_img.cols / 2, test_img.cols - 1));
	}if (imn == 0)
	{
		imgLeft = imread("C:\\camLeft.jpg", IMREAD_GRAYSCALE);
		imgRight = imread("C:\\camRight.jpg", IMREAD_GRAYSCALE);
		waitKey(33);
	}
	//-- 1. Read the images
	//Mat imgLeft = imread(argv[1], IMREAD_GRAYSCALE);
	//Mat imgRight = imread(argv[2], IMREAD_GRAYSCALE);
	//imshow("imgLeft", imgLeft);
	//imshow("imgRight", imgRight);
	//-- And create the image in which we will save our disparities
	Mat imgDisparity16S = Mat(imgLeft.rows, imgLeft.cols, CV_16S);
	Mat imgDisparity8U = Mat(imgLeft.rows, imgLeft.cols, CV_8UC1);
	Mat imgDisparity8U2 = Mat(imgLeft.rows, imgLeft.cols, CV_8UC2);

	if (imgLeft.empty() || imgRight.empty())
	{
		std::cout << " --(!) Error reading images " << std::endl; return -1;
	}

	//-- 2. Call the constructor for StereoBM
	//int ndisparities = 96;	/* 16 * 4;   /**< Range of disparity */
	//int SADWindowSize = 15; /**< Size of the block window. Must be odd */
	if (SADWindowSize % 2 == 0) {
		SADWindowSize++;
	}
	ndisparities = ndisparities * 16;

	Ptr<StereoBM> sbm = StereoBM::create(ndisparities, SADWindowSize);

	//-- 3. Calculate the disparity image
	sbm->compute(imgLeft, imgRight, imgDisparity16S);

	//-- Check its extreme values
	double minVal; double maxVal;

	minMaxLoc(imgDisparity16S, &minVal, &maxVal);

	printf("Min disp: %f Max value: %f \n", minVal, maxVal);

	//-- 4. Display it as a CV_8UC1 image
	imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255 / (maxVal - minVal));

	namedWindow(windowDisparity, WINDOW_NORMAL);
	imshow(windowDisparity, imgDisparity8U);

	//-- 5. Save the image
	//imwrite("SBM_sample.png", imgDisparity16S);


	return 0;
}
