/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include <libviso2/viso_stereo.h>
#include "../icu_calib/device.h"
#include "../icu_xml/xml_handler.cpp"

using namespace std;

int main () {


    //################ CAMERA SETTINGS #############################################################
    Device dev(DEV_MODE_WEBCAM, 0, 1);
    dev.setResolution(640, 480);
    cv::Mat frameL;
    cv::Mat frameR;

    //######################### STEREO SETTINGS ###############################################
    // set most important visual odometry parameters
    // for a full parameter list, look at: viso_stereo.h
    VisualOdometryStereo::parameters param;

    XML_handler xml("../icu_calib-build-desktop/camera_params.xml");
    //float new_baceline = xml.readParam<float>("baceline");
    cv::Mat translation;
    cv::Mat intrinsic;

    xml.readParam<double>("translation", translation);
    xml.readParam<double>("intrisitic_left", intrinsic);

    //xml.readParam<uint>("translation", new_trans);

    // calibration parameters for sequence 2010_03_09_drive_0019

    //cout << translation << endl;
    param.calib.f  = intrinsic.at<double>(1,1); // focal length in pixels
    param.calib.cu = intrinsic.at<double>(0,2); // principal point (u-coordinate) in pixels
    param.calib.cv = intrinsic.at<double>(1,2); // principal point (v-coordinate) in pixels
    param.base     = sqrt(pow(translation.at<double>(0, 0),2) + pow(translation.at<double>(1, 0),2) + pow(translation.at<double>(2, 0),2))/100; // baseline in meters

    //cout << "(f, u, v, bace)" << param.calib.f << " " << param.calib.cu << " " << param.calib.cv << " " << param.base << endl;

    // init visual odometry
    VisualOdometryStereo viso(param);

    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    Matrix pose = Matrix::eye(4);
    //########################################################################################

    char key;
    bool stop = false;

    //####################### MAIN LOOP ######################################################
    // loop through all frames i=0:372
    while (!stop) {

        if(!dev.getImage(DEV_SIDE_LEFT, frameL, DEV_COLOR_BW) || !dev.getImage(DEV_SIDE_RIGHT, frameR, DEV_COLOR_BW)){
            cout << "Can't capture :(((" << endl;
            continue;
        }

        key = cv::waitKey(20);
        if(key>=0) {
            stop = true;
        } else {

            int32_t width = frameL.cols;
            int32_t height = frameR.rows;
            // convert input images to uint8_t buffer
            uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
            uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
            int32_t k=0;
            for (int32_t v=0; v<height; v++) {
                const uint8_t* lim = frameL.ptr<uint8_t>(v);
                const uint8_t* rim = frameR.ptr<uint8_t>(v);
                for (int32_t u=0; u<width; u++) {
                    left_img_data[k]  = lim[u];
                    right_img_data[k] = rim[u];
                    k++;
                }
            }

                // compute visual odometry
            int32_t dims[] = {width,height,width};
            if (viso.process(left_img_data,right_img_data,dims)) {

                // on success, update current pose
                pose = pose * Matrix::inv(viso.getMotion());

                // output some statistics
                double num_matches = viso.getNumberOfMatches();
                double num_inliers = viso.getNumberOfInliers();
                cout << ", Matches: " << num_matches;
                cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
                cout << pose << endl << endl;

            } else {
                cout << " ... failed!" << endl;
            }

            // release uint8_t buffers
            free(left_img_data);
            free(right_img_data);


        }
    }
    //#########################################################################################

    // output
    cout << "Demo complete! Exiting ..." << endl;

    // exit
    return 0;
}
