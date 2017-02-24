#include <iostream>
#include <string.h>
#include <thread>

#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>

#include "ps4eye\ps4eye.h"
#include "features\features.h"
#include "stereo-match\stereo-match.h"
//#include "disparity\disparity.h"
//#include "stereo-calib\stereo_calib.h"

using namespace std;
using namespace cv;
using namespace ps4eye;

bool isRunning = true;
int ndisp = 6, wsize = 15, ndisp_max = 20, wsize_max = 100;

void on_trackbar(int, void*)
{
	cout << "ndisp : " << ndisp << " wsize : " << wsize << endl;
}

void update() {
	while (isRunning) {
		if (!ps4eye::PS4EYECam::updateDevices())
			break;
	}
}

int main() {
	Mat yuv, yuvL, yuvR, bgr, bgrL, bgrR, gbgrL, gbgrR;
	vector<Mat> Rbgr;

	ps4eye::PS4EYECam::PS4EYERef eye;

	uint8_t *frame_rgb_left;
	unsigned char * videoFrame;
	bool finish = false;
	eyeframe *frame;

	vector<PS4EYECam::PS4EYERef> devices(PS4EYECam::getDevices());

	if (devices.size() == 1) {
		eye = devices.at(0);
		eye->firmware_path = "firmware.bin";
		eye->firmware_upload();
		bool res = eye->init(1, 60);
		cout << res << endl;
		eye->start();
	}

	namedWindow("left");
	namedWindow("right");
	//namedWindow("RimgL", WINDOW_AUTOSIZE);
	//namedWindow("RimgR", WINDOW_AUTOSIZE);

	std::thread updateThread(update);


	//	eye->start_sensors_streaming();
	while (!finish) {
		if (eye) {
			bool isNewFrame = eye->isNewFrame();

			if (isNewFrame) {
				cout << isNewFrame << endl;

				eye->check_ff71();
				frame = eye->getLastVideoFramePointer();

				//		yuv.data = frame->videoLeftFrame;
				yuvL = Mat(eye->getHeight(), eye->getWidth(), CV_8UC2, frame->videoLeftFrame);
				cvtColor(yuvL, bgrL, COLOR_YUV2BGRA_YUY2);

				yuvR = Mat(eye->getHeight(), eye->getWidth(), CV_8UC2, frame->videoRightFrame);
				cvtColor(yuvR, bgrR, COLOR_YUV2BGRA_YUY2);

				imshow("left", bgrL);
				imshow("right", bgrR); 

				cvtColor(bgrL, gbgrL, CV_BGR2GRAY);	//color_mode
				cvtColor(bgrR, gbgrR, CV_BGR2GRAY);
				//imwrite("E:\\bgrL2.png", bgrL);
				//waitKey(30);
				//imwrite("E:\\bgrR2.png", bgrR);
				//waitKey(33);
				/*
				createTrackbar("ndisparities", "left", &ndisp, ndisp_max, on_trackbar);
				createTrackbar("SADWindowSize", "right", &wsize, wsize_max, on_trackbar);
				*/
				//Rbgr = testcalibrateStereoCamera(bgrL, bgrR);
				
				//stereomatch(bgrL, bgrR);
				get_features(bgrL, bgrR);
				
				// Calculate and show disparity map
				/*
				//if (disparity_map(bgrL, bgrR) != 0) {
				if (disparity_map(Rbgr[0], Rbgr[1], ndisp, wsize) != 0) {
					std::cout << "Disparity_Calc_Error" << std::endl;
					//eye->set_led_on();
				}
				*/
				//calibrateFromSavedImages2();
				//calibrateInRealTime(bgrL, bgrR);

				int keyCode =waitKey(1);
			}
		}

		else {
			finish = true;
		}
	}

	updateThread.join();

	//	eye->set_led_off();
	//	eye->stop_sensors_streaming();

	if (eye) {
		cout << "Shutdown begin wait..." << endl;
		//		eye->stop();
		eye->shutdown();
		//
	}

	return 0;
}