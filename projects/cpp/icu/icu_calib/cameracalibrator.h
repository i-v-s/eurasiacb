#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

class CameraCalibrator
{

    // input points:
    // the points in world coordinates
    vector<vector<cv::Point3f> > objectPoints;
    // the point positions in pixels
    vector<vector<cv::Point2f> > imagePoints;
    // output Matrices
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    // flag to specify how calibration is done
    int flag;
    // used in image undistortion
    cv::Mat map1,map2;
    bool mustInitUndistort;
    int success;
public:
    CameraCalibrator();
    bool addChessboardPoint(
            const cv::Mat& image,
            cv::Size & boardSize);
    void addPoints(const std::vector<cv::Point2f>&
            imageCorners, const std::vector<cv::Point3f>& objectCorners);
    double calibrate(cv::Size &imageSize);
    cv::Mat remap(const cv::Mat &image);

};

#endif // CAMERACALIBRATOR_H
