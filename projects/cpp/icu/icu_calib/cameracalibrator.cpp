#include "cameracalibrator.h"

CameraCalibrator::CameraCalibrator(cv::Size &bSize, float squareSize)
    : flag(0), mustInitUndistort(true), successes(0)
{
    boardSize = bSize;
    for (int i=0; i<boardSize.height; i++) {
        for (int j=0; j<boardSize.width; j++) {
           objectCorners.push_back(cv::Point3f(i*squareSize, j*squareSize, 0));
        }
    }

    cameraMatrixL = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrixR = cv::Mat::eye(3, 3, CV_64F);
}

bool CameraCalibrator::addChessboardPoint(const cv::Mat& limage, const cv::Mat& rimage)
{
    // the points on the chessboard
    cv::Mat images[2];
    images[0] = limage;
    images[1] = rimage;
    std::vector<cv::Point2f> imageCorners[2];

    string winName[2];
    winName[0] = "Left";
    winName[1] = "Right";

    bool displayCorners = true;
    const int maxScale = 2;

    bool rfound = true;
    bool found = false;

    for( int k = 0; k < 2; k++ )
    {
        std::vector<cv::Point2f>& corners = imageCorners[k];
        for( int scale = 1; scale <= maxScale; scale++ )
        {
            cv::Mat timg;
            if( scale == 1 )
                timg = images[k];
            else
                cv::resize(images[k], timg, cv::Size(), scale, scale);

            found = cv::findChessboardCorners(timg, boardSize, corners,
                            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
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
        rfound &= found;

        if( displayCorners )
        {
            if(found) {
                cv::Mat cimg, cimg1;
                cv::cvtColor(images[k], cimg, CV_GRAY2BGR);
                cv::drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(images[k].rows, images[k].cols);
                cv::resize(cimg, cimg1, cv::Size(), sf, sf);
                cv::imshow(winName[k], cimg1);
            } else {
                cv::imshow(winName[k], images[k]);
            }
        } else
            putwchar('.');
        if(found)
            cv::cornerSubPix(images[k], corners, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                  30, 0.01));

    }


    if(rfound)
        addPoints(imageCorners[0], imageCorners[1]);
    successes++;

    return rfound;
}

void CameraCalibrator::addPoints(const std::vector<cv::Point2f>& limageCorners,
                                 const std::vector<cv::Point2f>& rimageCorners)
{
    // 2D image points from one view
    imagePointsL.push_back(limageCorners);

    imagePointsR.push_back(rimageCorners);

    // corresponding 3D scene points
    objectPoints.push_back(objectCorners);
}

double CameraCalibrator::calibrate(cv::Size &imageSize)
{
    // undistorter must be reinitialized
    mustInitUndistort= true;

    // start calibration
    return stereoCalibrate(objectPoints, imagePointsL, imagePointsR,
                    cameraMatrixL, distCoeffsL,
                    cameraMatrixR, distCoeffsR,
                    imageSize, rot, trans, essen, fund,
                    cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);

}

void CameraCalibrator::show()
{
    cout << "Translation:" << endl << trans << endl << endl;
    cout << "Intrisitic Matrix (left):" << endl << cameraMatrixL << endl << endl;
    cout << "Intrisitic Matrix (right):" << endl << cameraMatrixR << endl << endl;
}

void CameraCalibrator::writeParams()
{
    XML_handler xml;
    xml.rootName("camera_params");
    //xml.addParam("baceline", 0.1234f);
    xml.addParam<double>("translation", trans);
    xml.addParam<double>("rotation", rot);
    xml.addParam<double>("intrinsic_left", cameraMatrixL);
    xml.addParam<double>("intrinsic_right", cameraMatrixR);
    xml.addParam<double>("distFactor_left", distCoeffsL);
    xml.addParam<double>("distFactor_right", distCoeffsR);
    xml.save("camera_params.xml");
}
