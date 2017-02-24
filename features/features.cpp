#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>

//using namespace std;
using namespace cv;

Mat imgRight_gray, imgLeft_gray;
int maxCorners = 50;

RNG rng(12345);
const char* source_windowL = "FeatureL";
const char* source_windowR = "FeatureR";

//int main(int argc, char** argv)
void get_features(Mat imgLeft, Mat imgRight)
{
	cvtColor(imgLeft, imgLeft_gray, COLOR_BGR2GRAY);
	cvtColor(imgRight, imgRight_gray, COLOR_BGR2GRAY);

	//
	/// Parameters for Shi-Tomasi algorithm
	std::vector<Point2f> cornersL;
	std::vector<Point2f> cornersR;
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;

	/// Copy the source image
	Mat copyL, copyR;
	copyL = imgLeft.clone();
	copyR = imgRight.clone();

	int64 t = getTickCount();

	/// Apply corner detection
	goodFeaturesToTrack(imgLeft_gray,
		cornersL,
		maxCorners,
		qualityLevel,
		minDistance,
		Mat(),
		blockSize,
		useHarrisDetector,
		k);
	goodFeaturesToTrack(imgRight_gray,
			cornersR,
			maxCorners,
			qualityLevel,
			minDistance,
			Mat(),
			blockSize,
			useHarrisDetector,
			k);
	t = getTickCount() - t;
	printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());

	/// Draw corners detected
	std::cout << "** Number of corners detected (L - R): " << cornersL.size() << "-" << cornersR.size() << std::endl;
	int r = 4;
	for (size_t i = 0; i < cornersL.size(); i++)
	{
		circle(copyL, cornersL[i], r, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1, 8, 0);
	}for (size_t i = 0; i < cornersR.size(); i++)
	{
		circle(copyR, cornersR[i], r, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1, 8, 0);
	}

	/// Show what you got
	namedWindow(source_windowL, WINDOW_AUTOSIZE);
	namedWindow(source_windowR, WINDOW_AUTOSIZE);
	imshow(source_windowL, copyL);
	imshow(source_windowR, copyR);
}
