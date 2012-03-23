#include "cameracalibrator.h"

CameraCalibrator::CameraCalibrator(cv::Size &bSize, float squareSize,
                                   bool useCalibrated, bool showRectified)
    : flag(0), mustInitUndistort(true), successes(0)
{
    boardSize = bSize;
    for (int i=0; i<boardSize.height; i++) {
        for (int j=0; j<boardSize.width; j++) {
           objectCorners.push_back(cv::Point3f(i*squareSize, j*squareSize, 0.f));
        }
    }

    useCalib = useCalibrated;
    showRect = showRectified;

    cameraMatrixL = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrixR = cv::Mat::eye(3, 3, CV_64F);
}

bool CameraCalibrator::addChessboardPoint(cv::Mat images[2])
{
    // the points on the chessboard
    std::vector<cv::Point2f> imageCorners[2];

    bool displayCorners = false;//true;
    const int maxScale = 2;

    bool rfound = true;
    bool found = false;
    for( int k = 0; k < 2; k++ )
    {
        vector<cv::Point2f>& corners = imageCorners[k];
        for( int scale = 1; scale <= maxScale; scale++ )
        {
            cv::Mat timg;
            if( scale == 1 )
                timg = images[k];
            else
                cv::resize(images[k], timg, cv::Size(), scale, scale);
            found = cv::findChessboardCorners(timg, boardSize, corners,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
            rfound &= found;
            if( found )
            {
                if( scale > 1 )
                {
                    cv::Mat cornersMat(corners);
                    cornersMat *= 1./scale;
                }
                break;
            }
        }
        if( displayCorners )
        {
            cv::Mat cimg, cimg1;
            cv::cvtColor(images[k], cimg, CV_GRAY2BGR);
            cv::drawChessboardCorners(cimg, boardSize, corners, found);
            double sf = 640./MAX(images[k].rows, images[k].cols);
            cv:resize(cimg, cimg1, cv::Size(), sf, sf);
            string camSide = "Left";
            if(k==1) {
                camSide = "Right";
            }
            cv::imshow(camSide, cimg1);
        }
        else
            putwchar('.');
        if( !found )
            break;
        cv::cornerSubPix(images[k], corners, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                  30, 0.01));
    }


    addPoints(imageCorners);
    successes++;

    return rfound;
}

void CameraCalibrator::addPoints(std::vector<cv::Point2f> imageCorners[2])
{
    // 2D image points from one view
    imagePointsL.push_back(imageCorners[0]);

    imagePointsR.push_back(imageCorners[1]);

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
                               rot, trans, essen, fund,
                               cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                           CV_CALIB_FIX_ASPECT_RATIO +
                                           CV_CALIB_ZERO_TANGENT_DIST +
                                           CV_CALIB_SAME_FOCAL_LENGTH);
}

void CameraCalibrator::show()
{
    cout << "Translation:" << endl << trans << endl << endl;
    cout << "Intrisitic Matrix (left):" << endl << cameraMatrixL << endl << endl;
    cout << "Intrisitic Matrix (right):" << endl << cameraMatrixR << endl << endl;

//    for(int i = 0; i < trans.rows; i++){
//        double* row = trans.ptr<double>(i);
//        for(int j = 0; j < trans.cols; j++)
//            cout<< row[j] << " ";
//        cout<<endl;
//    }

}


