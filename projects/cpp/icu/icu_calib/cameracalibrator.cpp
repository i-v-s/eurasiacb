#include "cameracalibrator.h"

CameraCalibrator::CameraCalibrator(cv::Size &bSize) : flag(0), mustInitUndistort(true), successes(0)
{
    boardSize = bSize;
    for (int i=0; i<boardSize.height; i++) {
        for (int j=0; j<boardSize.width; j++) {
           objectCorners.push_back(cv::Point3f(i, j, 0.0f));
        }
    }
}

bool CameraCalibrator::addChessboardPoint(const cv::Mat &imageL, const cv::Mat &imageR)
{
    // the points on the chessboard
    std::vector<cv::Point2f> imageCornersL;
    std::vector<cv::Point2f> imageCornersR;


    bool foundL = cv::findChessboardCorners(
               imageL, boardSize, imageCornersL);
    bool foundR = cv::findChessboardCorners(
               imageR, boardSize, imageCornersR);


    if(foundL && foundR){
        // Get subpixel accuracy on the corners
        cv::cornerSubPix(imageL, imageCornersL,
                        cv::Size(5,5),
                        cv::Size(-1,-1),
                        cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                        cv::TermCriteria::EPS,
                        30,
                        0.1)); // min accuracy
        cv::cornerSubPix(imageR, imageCornersR,
                        cv::Size(5,5),
                        cv::Size(-1,-1),
                        cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                        cv::TermCriteria::EPS,
                        30,
                        0.1)); // min accuracy
        //If we have a good board, add it to our data
        if ((imageCornersL.size() == boardSize.area())
                && (imageCornersL.size() == boardSize.area())) {
            // Add image and scene points from one view
            addPoints(imageCornersL, imageCornersR);
            successes++;
        }
    }

    return foundL&&foundR;
}

void CameraCalibrator::addPoints(const std::vector<cv::Point2f> &imageCornersL,
                                 const std::vector<cv::Point2f> &imageCornersR)
{
    // 2D image points from one view
    imagePointsL.push_back(imageCornersL);

    imagePointsR.push_back(imageCornersR);

    // corresponding 3D scene points
    objectPoints.push_back(objectCorners);
}

double CameraCalibrator::calibrate(cv::Size &imageSize)
{
    // undistorter must be reinitialized
    mustInitUndistort= true;
    //Output rotations and translations
    std::vector<cv::Mat> rvecs, tvecs;
    // start calibration
    return cv::stereoCalibrate(objectPoints, imagePointsL,
                               imagePointsR, cameraMatrixL, distCoeffsL,
                               cameraMatrixR, distCoeffsR, imageSize,
                               rot, trans, essen, fund);
//    calibrateCamera(objectPoints, // the 3D points
//                    imagePoints,
//                    imageSize,
//                    cameraMatrix,
//                    distCoeffs,
//                    rvecs, tvecs,
//                           flag);
}

void CameraCalibrator::showTrans()
{
    for(int i = 0; i < trans.rows; i++){
        double* row = trans.ptr<double>(i);
        for(int j = 0; j < trans.cols; j++)
            cout<< row[j] << " ";
        cout<<endl;
    }

}


