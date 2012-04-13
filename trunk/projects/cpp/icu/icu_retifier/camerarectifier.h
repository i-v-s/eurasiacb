#ifndef CAMERARECTIFIER_H
#define CAMERARECTIFIER_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <stdio.h>

#include "../icu_xml/xml_handler.cpp"


class CameraRectifier
{
private:
    // output Matrices
    cv::Mat cameraMatrixL;
    cv::Mat distCoeffsL;

    // output Matrices
    cv::Mat cameraMatrixR;
    cv::Mat distCoeffsR;

    cv::Mat rot;
    cv::Mat trans;

    cv::Size imSize;

    cv::Rect validRoi[2];
    cv::Mat rmap[2][2];

    cv::Mat canvas;
    double sf;
    int w, h;

    bool isVert;

    void init();

public:
    CameraRectifier(const cv::Mat& translation, const cv::Mat& rotation,
                    const cv::Mat& cameraMatrixLeft, const cv::Mat& distortionFactorLeft,
                    const cv::Mat& cameraMatrixRight, const cv::Mat& distortionFactorRight,
                    cv::Size imageSize);

    CameraRectifier(string filename, cv::Size imageSize);

    void rectify(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& result);

    double getBaceLine();
    cv::Mat getIntrinsicLeft();
    cv::Mat getIntrinsicRight();
    cv::Mat getTranslation();
};

#endif // CAMERARECTIFIER_H
