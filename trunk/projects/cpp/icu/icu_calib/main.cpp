#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include "cameracalibrator.h"

using namespace std;

int main()
{
    int verticalCornersNum = 9;
    int gorizonalCornersNum = 6;
    float squareSize = 2.5;

    std::vector<cv::Mat> images;
    //################ CAMERA SETTINGS #############################################################
    cv::VideoCapture captureL;
    cv::VideoCapture captureR;
    cv::Mat frameL;
    cv::Mat frameR;
    cv::Mat frameGrayL;
    cv::Mat frameGrayR;

    //camera initialization
    //TODO figure out how to get actual number of usbcam automatically
    captureL.open(1);
    captureR.open(2);
    //#########################################################################################
    //build windows
    cv::namedWindow("Left");
    cv::namedWindow("Right");


    char key;
    bool stop = false;


    cv::Size boardSize(gorizonalCornersNum, verticalCornersNum);

    CameraCalibrator ccal(boardSize, squareSize);
    cv::Size imSize(0,0);
    if(!captureL.read(frameL) ){
        cout << "Can't capture :(((" << endl;
        return 0;
    } else {
        imSize.height = frameL.rows;
        imSize.width = frameL.cols;
    }

    while (!stop) {

        key = cvWaitKey(20);

        if(!captureL.read(frameL) || !captureR.read(frameR)) {
            cout << "Can't capture :(((" << endl;
            continue;
        }

        cv::cvtColor(frameL, frameGrayL, CV_BGR2GRAY);
        cv::cvtColor(frameR, frameGrayR, CV_BGR2GRAY);

        if(key==32) {
                cv::imshow("Left", frameL);
                cv::imshow("Right", frameR);

                if(ccal.addChessboardPoint(frameGrayL, frameGrayR)) {
                    cout << "ok" << endl;
                } else {
                    cout << "can't find" << endl;
                }

        } else if (key == 10) {
            ccal.calibrate(imSize);
            ccal.show();
        } else if (key == 27) {
            stop = true;
        }
    }


    return 1;
}
