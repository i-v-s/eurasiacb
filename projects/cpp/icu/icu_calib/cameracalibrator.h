#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <stdio.h>

#include "../icu_xml/xml_handler.cpp"

using namespace std;

class CameraCalibrator
{

    // input points:
    // the points in world coordinates
    vector<vector<cv::Point3f> > objectPoints;
    // the point positions in pixels
    vector<vector<cv::Point2f> > imagePointsL;
    vector<vector<cv::Point2f> > imagePointsR;

    // output Matrices
    cv::Mat cameraMatrixL;
    cv::Mat distCoeffsL;

    // output Matrices
    cv::Mat cameraMatrixR;
    cv::Mat distCoeffsR;

    // flag to specify how calibration is done
    int flag;
    int successes;

    cv::Size boardSize;
    std::vector<cv::Point3f> objectCorners;
    bool mustInitUndistort;
    int success;

    cv::Mat rot;
    cv::Mat trans;
    cv::Mat fund;
    cv::Mat essen;

public:
    CameraCalibrator(cv::Size &bSize, float squareSize);
    bool addChessboardPoint(const cv::Mat& limage, const cv::Mat& rimage);
    void addPoints(const std::vector<cv::Point2f>& limageCorners, const std::vector<cv::Point2f>& rimageCorners);
    double calibrate(cv::Size &imageSize);
    void writeParams();
    void show();

};

#endif // CAMERACALIBRATOR_H
