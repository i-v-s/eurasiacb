#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include "cameracalibrator.h"

using namespace std;

int main()
{
    int verticalCornersNum = 7;
    int gorizonalCornersNum = 7;

    std::vector<cv::Mat>& images;
    //################ CAMERA SETTINGS #############################################################
    cv::VideoCapture captureL;
    //cv::VideoCapture captureR;
    cv::Mat frameL;
    //cv::Mat frameR;
    cv::Mat frameGrayL;
    //cv::Mat frameGrayR;

    //camera initialization
    //TODO figure out how to get actual number of usbcam automatically
    captureL.open(1);
    //captureR.open(1);
    //#########################################################################################
    //build windows
    cv::namedWindow("Left");


    char key;
    bool stop = false;

    CameraCalibrator ccal;
    cv::Size imSize(0,0);

    while (!stop) {

        key = cvWaitKey(30);

        if(!captureL.read(frameL) ){//||!captureR.read(frameR)) {
            cout << "Can't capture :(((" << endl;
            continue;
        }

        cv::cvtColor(frameL, frameGrayL, CV_BGR2GRAY);
        //cv::cvtColor(frameR, frameGrayR, CV_BGR2GRAY);
        imSize.height = frameL.rows;
        imSize.width = frameL.cols;

        if(key==15) {
                cv::Size boardSize(gorizonalCornersNum, verticalCornersNum);
                cv::imshow("Left", frameL);
                if(ccal.addChessboardPoint(frameGrayL, boardSize)) {
                    cout << "ok" << endl;
                } else {
                    cout << "can't find" << endl;
                }

        } else if (key = 10) {
            ccal.calibrate(imSize);
        } else if (key == 27) {
            stop = true;
        }
    }

    return 1;
}
