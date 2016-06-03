/* *************** Stereo Camera Calibration **************************
This code can be used to calibrate stereo cameras to get the intrinsic
and extrinsic files.
This code also generated rectified image, and also shows RMS Error and Reprojection error
to find the accuracy of calibration.
You can load saved stereo images or use this code to capture them in real time.
Keyboard Shortcuts for real time (ie clicking stereo image at run time):
1. Default Mode: Detecting (Which detects chessboard corners in real time)
2. 'c': Starts capturing stereo images (With 2 Sec gap, This can be changed by changing 'timeGap' macro)
3. 'p': Process and Calibrate (Once all the images are clicked you can press 'p' to calibrate)
Usage: StereoCameraCallibration [params]
--cam1 (value:0)                           Camera 1 Index
--cam2 (value:2)                           Camera 2 Index
--dr, --folder (value:.)                   Directory of images
-h, --height (value:6)                     Height of the board
--help (value:true)                        Prints this
--images, -n (value:40)                    No of stereo pair images
--post, --postfix (value:jpg)              Image extension. Ex: jpg,png etc
--prefixleft, --prel (value:image_left_)   Left image name prefix. Ex: image_left_
--prefixright, --prer (value:image_right_) Right image name postfix. Ex: image_right_
--realtime, --rt (value:1)                 Clicks stereo images before calibration. Use if you do not have stereo pair images saved
-w, --width (value:7)                      Width of the board
Example:   ./stereo_calib                                              Clicks stereo images at run time.
./stereo_calib -rt=0 -prel=left_ -prer=right_ -post=jpg     RealTime id off ie images should be loaded from disk. With images named left_1.jpg, right_1.jpg etc.
Cheers
*/

#include "stereo_calib.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdlib.h>

#define timeGap 3000000000U

using namespace cv;
using namespace std;

static void help() {
	cout << "/******** HELP *******/\n";
	cout << "\nThis program helps you to calibrate the stereo cameras.\n This program generates intrinsics.yml and extrinsics.yml which can be used in Stereo Matching Algorithms.\n";
	cout << "It also displays the rectified image\n";
	cout << "\nKeyboard Shortcuts for real time (ie clicking stereo image at run time):\n";
	cout << "1. Default Mode: Detecting (Which detects chessboard corners in real time)\n";
	cout << "2. 'c': Starts capturing stereo images (With 2 Sec gap, This can be changed by changing 'timeGap' macro)\n";
	cout << "3. 'p': Process and Calibrate (Once all the images are clicked you can press 'p' to calibrate)";
	cout << "\nType ./stereo_calib --help for more details.\n";
	cout << "\n/******* HELP ENDS *********/\n\n";
}

enum Modes { DETECTING, CAPTURING, CALIBRATING };
Modes mode = CAPTURING;
int noOfStereoPairs = 80;						//
int stereoPairIndex = 0, cornerImageIndex = 0;
int goIn = 1;
Mat _leftOri, _rightOri;
int64 prevTickCount;
vector<Point2f> cornersLeft, cornersRight;
vector<vector<Point2f> > cameraImagePoints[2];
Size boardSize = Size(6, 4);					//

string prefixLeft = "image_left_";				//
string prefixRight = "image_right_";			//
string postfix = "png";							//
string dir = "./images";								//

int calibType = 1;								//

Mat displayCapturedImageIndex(Mat);
Mat displayMode(Mat);
bool findChessboardCornersAndDraw(Mat, Mat);
void displayImages();
void saveImages(Mat, Mat, int);
void calibrateStereoCamera(Size);
void calibrateInRealTime(Mat, Mat);
void calibrateFromSavedImages(string, string, string, string);

vector<Mat> testcalibrateStereoCamera(Mat, Mat);

class stereocam {
public:

	FileStorage fsL;
	Mat cameraMatrix[2], distCoeffs[2];
	Mat R1, R2, P1, P2;
	Mat rmap[2][2];
	Size imageSize;

	stereocam() {
		cout << "Constructor called!" << endl;

		cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
		cameraMatrix[1] = Mat::eye(3, 3, CV_64F);

		fsL.open("camera_info/intrinsics.yml", FileStorage::READ);
		if (fsL.isOpened()) {
			fsL["M1"] >> cameraMatrix[0];
			fsL["D1"] >> distCoeffs[0];
			fsL["M2"] >> cameraMatrix[1];
			fsL["D2"] >> distCoeffs[1];
			fsL.release();
		}
		else
			cout << "Error: Could not open intrinsics file (intrinsic.yml)." << endl;
		//cout << "Reading extrinsics.yml - left/right.yaml "<< endl;
		cout << "M1:" << cameraMatrix[0] << endl;
		cout << "D1:" << distCoeffs[0] << endl;
		cout << "M2:" << cameraMatrix[1] << endl;
		cout << "D2:" << distCoeffs[1] << endl;
		FileStorage fsR;
		fsR.open("camera_info/extrinsics.yml", FileStorage::READ);
		if (fsR.isOpened()) {
			fsR["R1"] >> R1;
			fsR["R2"] >> R2;
			fsR["P1"] >> P1;
			fsR["P2"] >> P2;
			fsR.release();
		}
		else
			cout << "Error: Could not open extrinsics file (extrinsics.yml)." << endl;
		cout << "Cmaera parameters GOT !!" << endl;
		cout << "R1:" << R1 << endl;
		cout << "P1:" << P1 << endl;
		cout << "R2:" << R2 << endl;
		cout << "P2:" << P2 << endl;

		Mat crp = imread("./images/image_left_1.png");
		imageSize = crp.size();

		initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
	}
};
stereocam ps4eye;

Mat displayCapturedImageIndex(Mat img) {
	std::ostringstream imageIndex;
	imageIndex << stereoPairIndex << "/" << noOfStereoPairs;
	putText(img, imageIndex.str().c_str(), Point(50, 70), FONT_HERSHEY_PLAIN, 0.9, Scalar(0, 0, 255), 2);
	return img;
}

Mat displayMode(Mat img) {
	String modeString = "DETECTING";
	if (mode == CAPTURING) {
		modeString = "CAPTURING";
	}
	else if (mode == CALIBRATING) {
		modeString = "CALIBRATED";
	}
	putText(img, modeString, Point(50, 50), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
	if (mode == CAPTURING) {
		img = displayCapturedImageIndex(img);
	}
	return img;
}

bool findChessboardCornersAndDraw(Mat inputLeft, Mat inputRight) {
	_leftOri = inputLeft;
	_rightOri = inputRight;
	bool foundLeft = false, foundRight = false;
	cvtColor(inputLeft, inputLeft, COLOR_BGR2GRAY);
	cvtColor(inputRight, inputRight, COLOR_BGR2GRAY);
	foundLeft = findChessboardCorners(inputLeft, boardSize, cornersLeft, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
	foundRight = findChessboardCorners(inputRight, boardSize, cornersRight, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
	drawChessboardCorners(_leftOri, boardSize, cornersLeft, foundLeft);
	drawChessboardCorners(_rightOri, boardSize, cornersRight, foundRight);
	_leftOri = displayMode(_leftOri);
	_rightOri = displayMode(_rightOri);
	if (foundLeft && foundRight) {
		return true;
	}
	else {
		return false;
	}
}

void displayImages() {
	imshow("Left Image", _leftOri);
	imshow("Right Image", _rightOri);
}

void saveImages(Mat leftImage, Mat rightImage, int pairIndex) {
	cameraImagePoints[0].push_back(cornersLeft);
	cameraImagePoints[1].push_back(cornersRight);
	if (calibType == 1) {
		cvtColor(leftImage, leftImage, COLOR_BGR2GRAY);
		cvtColor(rightImage, rightImage, COLOR_BGR2GRAY);
		std::ostringstream leftString, rightString;
		leftString << dir << "/" << prefixLeft << pairIndex << "." << postfix;
		rightString << dir << "/" << prefixRight << pairIndex << "." << postfix;
		imwrite(leftString.str().c_str(), leftImage);
		imwrite(rightString.str().c_str(), rightImage);
		cout << "----------------------Image Write SUCESS!!" << endl;
	}
}

void calibrateStereoCamera(Size imageSize) {
	vector<vector<Point3f> > objectPoints;
	objectPoints.resize(noOfStereoPairs);
	for (int i = 0; i<noOfStereoPairs; i++) {
		for (int j = 0; j<boardSize.height; j++) {
			for (int k = 0; k<boardSize.width; k++) {
				objectPoints[i].push_back(Point3f(float(j), float(k), 0.0));
			}
		}
	}
	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	Mat R, T, E, F;
	double rms = stereoCalibrate(objectPoints, cameraImagePoints[0], cameraImagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO +
		CALIB_ZERO_TANGENT_DIST +
		CALIB_SAME_FOCAL_LENGTH +
		CALIB_RATIONAL_MODEL +
		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	cout << "RMS Error: " << rms << "\n";
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (int i = 0; i < noOfStereoPairs; i++)
	{
		int npt = (int)cameraImagePoints[0][i].size();
		Mat imgpt[2];
		for (int k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(cameraImagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (int j = 0; j < npt; j++)
		{
			double errij = fabs(cameraImagePoints[0][i][j].x*lines[1][j][0] +
				cameraImagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(cameraImagePoints[1][i][j].x*lines[0][j][0] +
					cameraImagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "Average Reprojection Error: " << err / npoints << endl;
	FileStorage fs("intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: Could not open intrinsics file.";
	Mat R1, R2, P1, P2, Q;
	Rect validROI[2];
	stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 1, imageSize, &validROI[0], &validROI[1]);
	fs.open("extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: Could not open extrinsics file";
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
	Mat rmap[2][2];
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo) {
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else {
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}
	String file;
	namedWindow("rectified");
	for (int i = 0; i < noOfStereoPairs; i++) {
		for (int j = 0; j < 2; j++) {
			if (j == 0) {
				file = prefixLeft;
			}
			else if (j == 1) {
				file = prefixRight;
			}
			ostringstream st;
			st << dir << "/" << file << i + 1 << "." << postfix;
			Mat img = imread(st.str().c_str()), rimg, cimg;
			remap(img, rimg, rmap[j][0], rmap[j][1], INTER_LINEAR);
			cimg = rimg;
			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*j, 0, w, h)) : canvas(Rect(0, h*j, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
			Rect vroi(cvRound(validROI[j].x*sf), cvRound(validROI[j].y*sf),
				cvRound(validROI[j].width*sf), cvRound(validROI[j].height*sf));
			rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
		}
		if (!isVerticalStereo)
			for (int j = 0; j < canvas.rows; j += 16)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (int j = 0; j < canvas.cols; j += 16)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
	}
}

void calibrateInRealTime(Mat inputLeft, Mat inputRight) {
	//VideoCapture camLeft(cam1), camRight(cam2);
	if (inputLeft.empty() || inputRight.empty() ) {
		cout << "Error: Stereo Cameras || Frames not found or there is some problem connecting them. Please check your cameras.\n";
		exit(-1);
	}
	Mat  copyImageLeft, copyImageRight;
	bool foundCornersInBothImage = false;
	
	//for (; ; ) {
		//camLeft >> inputLeft;
		//camRight >> inputRight;
		if ((inputLeft.rows != inputRight.rows) || (inputLeft.cols != inputRight.cols)) {
			cout << "Error: Images from both cameras are not of same size. Please check the size of each camera.\n";
			exit(-1);
		}
		inputLeft.copyTo(copyImageLeft);
		inputRight.copyTo(copyImageRight);
		foundCornersInBothImage = findChessboardCornersAndDraw(inputLeft, inputRight);
		if (foundCornersInBothImage && mode == CAPTURING && stereoPairIndex<noOfStereoPairs) {
			cout << "##------- Corners Found" << endl;
			//int64 thisTick = getTickCount();
			//int64 diff = thisTick - prevTickCount;
			//if (goIn == 1 || diff >= timeGap) {
			//	goIn = 0;
				saveImages(copyImageLeft, copyImageRight, ++stereoPairIndex);
				cout << "-- Saving image..." << endl;

			//	prevTickCount = getTickCount();
			//}
		}
		displayImages();
		if (mode == CALIBRATING) {
			calibrateStereoCamera(inputLeft.size());
			waitKey();
		}
		char keyBoardInput = (char)waitKey(50);
		if (keyBoardInput == 'q' || keyBoardInput == 'Q') {
			exit(-1);
		}
		else if (keyBoardInput == 'c' || keyBoardInput == 'C') {
			mode = CAPTURING;
		}
		else if (keyBoardInput == 'p' || keyBoardInput == 'P') {
			mode = CALIBRATING;
		}
	//}
}

void calibrateFromSavedImages2() {
	calibrateFromSavedImages(dir, prefixLeft, prefixRight, postfix);
}
void calibrateFromSavedImages(string dr, string prel, string prer, string post) {
	Size imageSize;
	for (int i = 0; i<noOfStereoPairs; i++) {
		Mat inputLeft, inputRight, copyImageLeft, copyImageRight;
		ostringstream imgIndex;
		imgIndex << i + 1;
		bool foundCornersInBothImage = false;
		string sourceLeftImagePath, sourceRightImagePath;
		sourceLeftImagePath = dr + "/" + prel + imgIndex.str() + "." + post;
		sourceRightImagePath = dr + "/" + prer + imgIndex.str() + "." + post;
		inputLeft = imread(sourceLeftImagePath);
		inputRight = imread(sourceRightImagePath);
		imageSize = inputLeft.size();
		if (inputLeft.empty() || inputRight.empty()) {
			cout << "\nCould no find image: " << sourceLeftImagePath << " or " << sourceRightImagePath << ". Skipping images.\n";
			continue;
		}
		if ((inputLeft.rows != inputRight.rows) || (inputLeft.cols != inputRight.cols)) {
			cout << "\nError: Left and Right images are not of some size. Please check the size of the images. Skipping Images.\n";
			continue;
		}
		inputLeft.copyTo(copyImageLeft);
		inputRight.copyTo(copyImageRight);
		foundCornersInBothImage = findChessboardCornersAndDraw(inputLeft, inputRight);
		if (foundCornersInBothImage && stereoPairIndex<noOfStereoPairs) {
			saveImages(copyImageLeft, copyImageRight, ++stereoPairIndex);
		}
		displayImages();
	}
	if (stereoPairIndex > 2) {
		calibrateStereoCamera(imageSize);
		waitKey();
	}
	else {
		cout << "\nInsufficient stereo images to calibrate.\n";
	}
}

int stereo_calib(int argc, char** argv) {
	help();
	const String keys =
		"{help| |Prints this}"
		"{h height|6|Height of the board}"
		"{w width|7|Width of the board}"
		"{rt realtime|1|Clicks stereo images before calibration. Use if you do not have stereo pair images saved}"
		"{n images|40|No of stereo pair images}"
		"{dr folder|.|Directory of images}"
		"{prel prefixleft|image_left_|Left image name prefix. Ex: image_left_}"
		"{prer prefixright|image_right_|Right image name postfix. Ex: image_right_}"
		"{post postfix|jpg|Image extension. Ex: jpg,png etc}"
		"{cam1|0|Camera 1 Index}"
		"{cam2|2|Camera 2 Index}";
	CommandLineParser parser(argc, argv, keys);
	if (parser.has("help")) {
		parser.printMessage();
		exit(-1);
	}
	boardSize = Size(parser.get<int>("w"), parser.get<int>("h"));
	noOfStereoPairs = parser.get<int>("n");
	prefixLeft = parser.get<string>("prel");
	prefixRight = parser.get<string>("prer");
	postfix = parser.get<string>("post");
	dir = parser.get<string>("dr");
	calibType = parser.get<int>("rt");
	namedWindow("Left Image");
	namedWindow("Right Image");
	switch (calibType) {
	case 0:
		calibrateFromSavedImages(dir, prefixLeft, prefixRight, postfix);
		break;
	//case 1:
	//	calibrateInRealTime(parser.get<int>("cam1"), parser.get<int>("cam2"));
	//	break;
	default:
		cout << "-rt should be 0 or 1. Ex: -rt=1\n";
		break;
	}
	return 0;
}


vector<Mat> testcalibrateStereoCamera(Mat imgL, Mat imgR) {

	//Mat imgL = imread("./images/image_left_1.png");
	//Mat imgR = imread("./images/image_right_1.png");
	//Size imageSizeL = imgL.size();
	//Size imageSizeR = imgR.size();
	/*
	namedWindow("imgL1");
	namedWindow("imgR1");
	imshow("imgL1", imgL);
	waitKey(33);
	imshow("imgR1", imgR);
	waitKey(33);
	*/
	/*
	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	Mat R1, R2, P1, P2, Q;

	//cout << "Reading Intrinsic.yml - left/right.yaml "<< endl;
	FileStorage fsL;
	fsL.open("camera_info/intrinsics.yml", FileStorage::READ);
	if (fsL.isOpened()) {
		fsL["M1"] >> cameraMatrix[0];
		fsL["D1"] >> distCoeffs[0];
		fsL["M2"] >> cameraMatrix[1];
		fsL["D2"] >> distCoeffs[1];
		fsL.release();
	}
	else
		cout << "Error: Could not open intrinsics file (intrinsic.yml)." << endl;
	//cout << "Reading extrinsics.yml - left/right.yaml "<< endl;
	cout << "M1:" << cameraMatrix[0] << endl;
	cout << "D1:" << distCoeffs[0] << endl;
	cout << "M2:" << cameraMatrix[1] << endl;
	cout << "D2:" << distCoeffs[1] << endl;
	FileStorage fsR;
	fsR.open("camera_info/extrinsics.yml", FileStorage::READ);
	if (fsR.isOpened()) {
		fsR["R1"] >> R1;
		fsR["R2"] >> R2;
		fsR["P1"] >> P1;
		fsR["P2"] >> P2;
		fsR.release();
	}
	else
		cout << "Error: Could not open extrinsics file (extrinsics.yml)." << endl;
	cout << "Cmaera parameters GOT !!" << endl;
	cout << "R1:" << R1 << endl;
	cout << "P1:" << P1 << endl;
	cout << "R2:" << R2 << endl;
	cout << "P2:" << P2 << endl;
	
	Mat rmap[2][2];
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSizeL, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSizeR, CV_16SC2, rmap[1][0], rmap[1][1]);
	*/
	remap(imgL, imgL, ps4eye.rmap[0][0], ps4eye.rmap[0][1], INTER_LINEAR);
	remap(imgR, imgR, ps4eye.rmap[1][0], ps4eye.rmap[1][1], INTER_LINEAR);

	vector<Mat> rimg;
	rimg.push_back(imgL.clone());
	rimg.push_back(imgR.clone());
	/*
	namedWindow("imgL", WINDOW_AUTOSIZE);
	namedWindow("imgR", WINDOW_AUTOSIZE);
	imshow("imgL", rimg[0]);
	imshow("imgR", rimg[1]);

	cout << "rectified!" << endl;
	waitKey(0);
	*/

	return rimg;
}