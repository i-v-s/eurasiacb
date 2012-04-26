#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "camerarectifier.h"
#include "../icu_calib/device.h"

int main()
{
    Device dev(DEV_MODE_WEBCAM, 0, 1);
    dev.setResolution(640, 480);
    cv::Mat frameL, frameR;
    cv::Mat image;

    cv::namedWindow("Rectificated");

    CameraRectifier camrect("../icu_calib-build-desktop/camera_params.xml", cv::Size(640, 480));

    bool stop = false;
    char key;

    while (!stop) {

        if(!dev.getImages(frameL, frameR, DEV_COLOR_BW)){
            cout << "Can't capture :(((" << endl;
            continue;
        }

        key = cv::waitKey(20);
        if(key>=0) {
            stop = true;
        } else {
            camrect.rectify(frameL, frameR, image);
            cv::imshow("Rectificated", image);

        }
    }

    return 1;
}
