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
        captures[DEV_SIDE_LEFT].open(CV_CAP_OPENNI+usb_num_left);
        captures[DEV_SIDE_LEFT].set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ ); // default

        captures[DEV_SIDE_RIGHT].open(CV_CAP_OPENNI+usb_num_right);
        captures[DEV_SIDE_RIGHT].set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ ); // default
    }
}

bool Device::getImage(int side, cv::Mat& image, int color_mode=DEV_COLOR_BW)
{

    if(mode == DEV_MODE_WEBCAM) {
        if(!captures[side].read(image))
            return false;
        if(color_mode == DEV_COLOR_BW)
            cv::cvtColor(image, image, CV_BGR2GRAY);
    } else if (mode == DEV_MODE_KINECT) {
        if( !captures[side].grab() )
        {
            return false;
        } else {
            if(color_mode == DEV_COLOR_BW)
               captures[side].retrieve( image, CV_CAP_OPENNI_GRAY_IMAGE );
            else
               captures[side].retrieve( image, CV_CAP_OPENNI_BGR_IMAGE );
        }
    }

    return true;
}

bool Device::getSize(cv::Size &size)
{
    size.width = captures[DEV_SIDE_LEFT].get( CV_CAP_PROP_FRAME_WIDTH);
    size.height = captures[DEV_SIDE_LEFT].get( CV_CAP_PROP_FRAME_HEIGHT);

    return true;
}

void Device::setResolution(double width, double height)
{
    captures[DEV_SIDE_LEFT].set(CV_CAP_PROP_FRAME_WIDTH, width);
    captures[DEV_SIDE_LEFT].set(CV_CAP_PROP_FRAME_HEIGHT, height);

    captures[DEV_SIDE_RIGHT].set(CV_CAP_PROP_FRAME_WIDTH, width);
    captures[DEV_SIDE_RIGHT].set(CV_CAP_PROP_FRAME_HEIGHT, height);
}

bool Device::getImages(cv::Mat &imageLeft, cv::Mat &imageRight, int color_mode)
{
    int ok = true;
    ok &= getImage(DEV_SIDE_LEFT, imageLeft, color_mode);
    ok &= getImage(DEV_SIDE_RIGHT, imageRight, color_mode);
    return ok;
}
