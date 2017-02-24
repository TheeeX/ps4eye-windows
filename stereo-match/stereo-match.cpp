/*
 *  stereo_match.cpp
 *	disparity map using :
 *		STEREO_BM, STEREO_SGBM, 
 *		STEREO_HH, STEREO_VAR, 
 *		STEREO_3WAY 
 *  Author : Karan(TheeeX)
 *
 */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include <iostream>

//using namespace std;
using namespace cv;

std::string img1_filename = "C:\\ps4eye\\images\\image_left_1.png";
std::string img2_filename = "C:\\ps4eye\\images\\image_right_1.png";
std::string intrinsic_filename = "C:\\ps4eye\\camera_info\\intrinsics.yml";
std::string extrinsic_filename = "C:\\ps4eye\\camera_info\\extrinsics.yml";
std::string disparity_filename = "E:\\dc.png";
std::string point_cloud_filename = "E:\\pc.png";

enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
int alg = STEREO_3WAY;	// STEREO_3WAY, STEREO_VAR, STEREO_HH, STEREO_SGBM;
std::string _alg = "sgbm3way";	//"sgbm3way";
int SADWindowSize = 15, numberOfDisparities = 96;
bool no_display = false;
float scale = 1.0;
Mat map11, map12, map21, map22;
Mat img1r, img2r;

Ptr<StereoBM> bm = StereoBM::create(16, 9);
Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

class stereocam1 {
public:

	FileStorage fsL;
	Mat M1, M2, D1, D2;
	Mat R1, R2, P1, P2, R, T, Q;
	Mat rmap[2][2];
	Size imageSize;

	stereocam1() {
		printf("Constructor called!\n");

		M1 = Mat::eye(3, 3, CV_64F);
		M2 = Mat::eye(3, 3, CV_64F);

		fsL.open(intrinsic_filename, FileStorage::READ);
		if (fsL.isOpened()) {
			fsL["M1"] >> M1;
			fsL["D1"] >> D1;
			fsL["M2"] >> M2;
			fsL["D2"] >> D2;
			fsL.release();
		}
		else
			std::cout << "Error: Could not open intrinsics file (intrinsic.yml)." << std::endl;
		//std::cout << "Reading extrinsics.yml - left/right.yaml "<< std::endl;
		std::cout << "M1:" << M1 << std::endl;
		std::cout << "D1:" << D1 << std::endl;
		std::cout << "M2:" << M2 << std::endl;
		std::cout << "D2:" << D2 << std::endl;
		FileStorage fsR;
		fsR.open(extrinsic_filename, FileStorage::READ);
		if (fsR.isOpened()) {
			fsR["R1"] >> R1;
			fsR["R2"] >> R2;
			fsR["P1"] >> P1;
			fsR["P2"] >> P2;
			fsR["R"] >> R;
			fsR["T"] >> T;
			fsR["Q"] >> Q;
			fsR.release();
		}
		else
			std::cout << "Error: Could not open extrinsics file (extrinsics.yml)." << std::endl;
		std::cout << "Cmaera parameters GOT !!" << std::endl;
		std::cout << "R1:" << R1 << std::endl;
		std::cout << "P1:" << P1 << std::endl;
		std::cout << "R2:" << R2 << std::endl;
		std::cout << "P2:" << P2 << std::endl;

		Mat crp = imread("C:\\ps4eye\\images\\image_left_1.png");
		imageSize = crp.size();

		//initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		//initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);


		initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
		std::cout << "Done initdist..." << std::endl;
	}
};
stereocam1 ps4cam;

static void print_help()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|sgbm3way] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i=<intrinsic_filename>] [-e=<extrinsic_filename>]\n"
           "[--no-display] [-o=<disparity_image>] [-p=<point_cloud_file>]\n");
}
/*
stereo-match C:\ps4eye\images\image_left_1.png C:\ps4eye\images\image_right_1.png 
			--algorithm=bm --blocksize=15 --max-disparity=96 --scale=1.f
			-i=C:\ps4eye\camera_info\intrinsics.yml -e=C:\ps4eye\camera_info\extrinsics.yml
			-o=./disparity.png -p=./pc.png
*/

static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

//int main(int argc, char** argv)
//int stereomatch(Mat imgLeft, Mat imgRight)
int stereomatch(Mat img1, Mat img2)
{
	alg = _alg == "bm" ? STEREO_BM :
		_alg == "sgbm" ? STEREO_SGBM :
		_alg == "hh" ? STEREO_HH :
		_alg == "var" ? STEREO_VAR :
		_alg == "sgbm3way" ? STEREO_3WAY : -1;

    if( alg < 0 )
    {
        printf("Command-line parameter error: Unknown stereo algorithm\n\n");
        print_help();
        return -1;
    }
    if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
    {
        printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
        print_help();
        return -1;
    }
    if (scale < 0)
    {
        printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
        return -1;
    }
    if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
    {
        printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
        return -1;
    }
    if(img1.empty() || img2.empty() )
    {
        printf("Command-line parameter error: both left and right images must be specified\n");
        return -1;
    }
    if( (!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()) )
    {
        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }

    if( extrinsic_filename.empty() && !point_cloud_filename.empty() )
    {
        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }

    int color_mode = alg == STEREO_BM ? 0 : -1;
	/*
	Mat img1, img2;
	cvtColor(imgLeft, img1, CV_BGR2GRAY);	//color_mode
	cvtColor(imgRight, img2, CV_BGR2GRAY);
	*/
    if (img1.empty())
    {
        printf("Command-line parameter error: could not load the first input image file\n");
        return -1;
    }
    if (img2.empty())
    {
        printf("Command-line parameter error: could not load the second input image file\n");
        return -1;
    }
	
    if (scale != 1.f)
    {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(img1, temp1, Size(), scale, scale, method);
        img1 = temp1;
        resize(img2, temp2, Size(), scale, scale, method);
        img2 = temp2;
    }
	
    Size img_size = img1.size();

    Rect roi1, roi2;
    Mat Q;
	
	std::cout << "#2 --> remap (...)" << std::endl;

	remap(img1, img1, ps4cam.rmap[0][0], ps4cam.rmap[0][1], INTER_LINEAR);
	remap(img2, img2, ps4cam.rmap[1][0], ps4cam.rmap[1][1], INTER_LINEAR);

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = img1.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    if(alg==STEREO_HH)
        sgbm->setMode(StereoSGBM::MODE_HH);
    else if(alg==STEREO_SGBM)
        sgbm->setMode(StereoSGBM::MODE_SGBM);
    else if(alg==STEREO_3WAY)
        sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

    Mat disp, disp8;
    //Mat img1p, img2p, dispp;
    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    int64 t = getTickCount();
    if( alg == STEREO_BM )
        bm->compute(img1, img2, disp);
    else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY )
        sgbm->compute(img1, img2, disp);
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
	if (alg != STEREO_VAR) {
			disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
			//disp = dispp.colRange(numberOfDisparities, img1p.cols);
	}
    else
        disp.convertTo(disp8, CV_8U);
    if( !no_display )
    {
        namedWindow("left", 1);
        imshow("left", img1);
        namedWindow("right", 1);
        imshow("right", img2);
        namedWindow("disparity", 0);
        imshow("disparity", disp8);
        //printf("press any key to continue...");
        fflush(stdout);
        waitKey(30);
        printf("\n");
    }
	/*
    if(!disparity_filename.empty())
        imwrite(disparity_filename, disp8);

    if(!point_cloud_filename.empty())
    {
        printf("storing the point cloud...");
		stereoRectify(ps4cam.M1, ps4cam.D1, ps4cam.M2, ps4cam.D2, img_size, ps4cam.R, ps4cam.T, ps4cam.R1, ps4cam.R2, ps4cam.P1, ps4cam.P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
        fflush(stdout);
        Mat xyz;
        reprojectImageTo3D(disp, xyz, Q, true);
        saveXYZ(point_cloud_filename.c_str(), xyz);
        printf("\n");
    }
	*/
	return 0;
}
