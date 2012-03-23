#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

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

    bool useCalib;
    bool showRect;

public:
    CameraCalibrator(cv::Size &bSize, float squareSize,
                     bool useCalibrated = true, bool showRectified = true);
    bool addChessboardPoint(
                   cv::Mat images[2]);
    void addPoints(std::vector<cv::Point2f> imageCorners[2]);
    double calibrate(cv::Size &imageSize);
    void show();

};

#endif // CAMERACALIBRATOR_H
