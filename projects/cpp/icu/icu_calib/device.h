#ifndef DEVICE_H
#define DEVICE_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "kinect.h"

#define DEV_MODE_WEBCAM 0
#define DEV_MODE_KINECT 1

#define DEV_SIDE_LEFT 0
#define DEV_SIDE_RIGHT 1

#define DEV_COLOR_BW 0
#define DEV_COLOR_RGB 1

class Device
{
public:
    Device(int c_mode, int usb_num_left, int usb_num_right);

    bool getImage(int side, cv::Mat& image,int color_mode);
    bool getSize(cv::Size& size);

private:
    int mode;
    cv::VideoCapture captures[2];

    //Kinect dev;
};

#endif // DEVICE_H
