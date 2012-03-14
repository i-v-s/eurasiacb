#include "device.h"

Device::Device(int c_mode, int usb_num_left, int usb_num_right)
{
    mode = c_mode;

    if(mode == DEV_MODE_WEBCAM){
        //camera initialization
        //TODO figure out how to get actual number of usbcam automatically
        captures[DEV_SIDE_LEFT].open(usb_num_left);
        captures[DEV_SIDE_RIGHT].open(usb_num_right);
    } else if (mode == DEV_MODE_KINECT) {
        //freenect.createDevice<Kinect>(usb_num_left);
        //dev.startVideo();
        //devs[DEV_SIDE_RIGHT] = freenect.createDevice<MyFreenectDevice>(usb_num_right);
        //devs[DEV_SIDE_RIGHT].startVideo();
    }
}

bool Device::getImage(int side, cv::Mat& image, int color_mode=DEV_COLOR_BW)
{

    if(mode == DEV_MODE_WEBCAM) {
        if(!captures[side].read(image))
            return false;
    } else if (mode == DEV_MODE_KINECT) {
        //if(!devs.getVideo(image))
        //    return false;
    }

    if(color_mode == DEV_COLOR_BW)
        cv::cvtColor(image, image, CV_BGR2GRAY);
    return true;
}

bool Device::getSize(cv::Size &size)
{
    cv::Mat image;
    if(!getImage(DEV_SIDE_LEFT, image))
        return false;
    size.width = image.cols;
    size.height = image.rows;

    return true;
}
