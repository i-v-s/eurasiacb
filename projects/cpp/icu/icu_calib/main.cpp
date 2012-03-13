#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include "cameracalibrator.h"

#include "device.h"

using namespace std;

int main()
{
    int verticalCornersNum = 9;
    int gorizonalCornersNum = 6;
    float squareSize = 2.5;

    cv::namedWindow("Left");
    cv::namedWindow("Right");

    Device dev(DEV_MODE_WEBCAM, 1, 0);

    char key;
    bool stop = false;


    cv::Size boardSize(gorizonalCornersNum, verticalCornersNum);

    CameraCalibrator ccal(boardSize, squareSize);
    cv::Size imSize(0,0);

    if(!dev.getSize(imSize)){
        cout << "Can't capture :(((" << endl;
        return 0;
    }

    cv::Mat frameL;
    cv::Mat frameR;

    while (!stop) {

        key = cvWaitKey(20);

        if(dev.getImage(DEV_SIDE_LEFT, frameL, DEV_COLOR_BW)  && dev.getImage(DEV_SIDE_RIGHT, frameR, DEV_COLOR_BW)) {

            if(key==32) {
                    cv::imshow("Left", frameL);
                    //cv::imshow("Right", frameR);

                    /*
                    if(ccal.addChessboardPoint(frameL, frameR)) {
                        cout << "ok" << endl;
                    } else {
                        cout << "can't find" << endl;
                    }
                    */

            } else if (key == 10) {
                ccal.calibrate(imSize);
                ccal.show();
            } else if (key == 27) {
                stop = true;
            }
        } else {
            cout << "Can't capture :(((" << endl;
        }
    }


    return 1;
}
