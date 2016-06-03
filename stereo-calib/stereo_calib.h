#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

void calibrateInRealTime(Mat, Mat);
void calibrateFromSavedImages2();
std::vector<Mat> testcalibrateStereoCamera(Mat, Mat);