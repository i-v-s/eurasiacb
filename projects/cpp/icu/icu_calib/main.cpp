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
    int verticalCornersNum = 7;
    int gorizonalCornersNum = 7;
    float squareSize = 4.f;

    Device dev(DEV_MODE_WEBCAM, 0, 1);

    char key;
    bool stop = false;


    cv::Size boardSize(gorizonalCornersNum, verticalCornersNum);

    CameraCalibrator ccal(boardSize, squareSize);
    cv::Size imSize(0,0);

    if(!dev.getSize(imSize)){
        cout << "Can't capture :(((" << endl;
        return 0;
    }

    cout<<imSize.width << endl;

    cv::Mat frame[2];
    cv::namedWindow("Left");
    cv::namedWindow("Right");

    while (!stop) {

        key = cvWaitKey(100);

        if(dev.getImage(DEV_SIDE_LEFT, frame[0], DEV_COLOR_BW) && dev.getImage(DEV_SIDE_RIGHT, frame[1], DEV_COLOR_BW)) {

           if(key==32) {
                    //cv::imshow("Left", frameL);
                    //cv::imshow("Right", frameR);
               cout << "i'm here" <<  endl;

                if(ccal.addChessboardPoint(frame)) {
                    cout << "ok" << endl;
                } else {
                    cout << "can't find" << endl;
                }

            } else if (key == 10) {
                ccal.calibrate(imSize);
                ccal.show();
            }
        } else {
            cout << "Can't capture :(((" << endl;
        }

        if (key == 27) {
            stop = true;
        }
    }


    return 1;
}
