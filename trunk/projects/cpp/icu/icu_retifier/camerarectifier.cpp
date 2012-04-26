#include "camerarectifier.h"

CameraRectifier::CameraRectifier(const cv::Mat& translation, const cv::Mat& rotation,
                                 const cv::Mat& cameraMatrixLeft, const cv::Mat& distortionFactorLeft,
                                 const cv::Mat& cameraMatrixRight, const cv::Mat& distortionFactorRight,
                                 cv::Size imageSize)
{

    // output Matrices
    cameraMatrixL = cameraMatrixLeft;
    distCoeffsL = distortionFactorLeft;

    // output Matrices
    cameraMatrixR = cameraMatrixRight;
    distCoeffsR = distortionFactorRight;

    rot = rotation;
    trans = translation;

    imSize = imageSize;

    init();
}

CameraRectifier::CameraRectifier(string filename, cv::Size imageSize)
{
    XML_handler xml(filename);
    xml.readParam<double>("translation", trans);
    xml.readParam<double>("rotation", rot);
    xml.readParam<double>("intrinsic_left", cameraMatrixL);
    xml.readParam<double>("intrinsic_right", cameraMatrixR);
    xml.readParam<double>("distFactor_left", distCoeffsL);
    xml.readParam<double>("distFactor_right", distCoeffsR);

//    distCoeffsL.at<double>(0,0) = 0.0;
//    distCoeffsL.at<double>(0,1) = 0.0;
//    distCoeffsL.at<double>(0,7) = 0.0;
//    distCoeffsR.at<double>(0,0) = 0.0;
//    distCoeffsR.at<double>(0,1) = 0.0;
//    distCoeffsR.at<double>(0,7) = 0.0;

//    std::cout << "trans" << std::endl;
//    std::cout << trans << std::endl;
//    std::cout << "rotation" << std::endl;
//    std::cout << rot << std::endl;
//    std::cout << "intrinsic_left" << std::endl;
//    std::cout << cameraMatrixL << std::endl;
//    std::cout << "intrinsic_right" << std::endl;
//    std::cout << cameraMatrixR << std::endl;
    std::cout << "distFactor_left" << std::endl;
    std::cout << distCoeffsL << std::endl;
    std::cout << "distFactor_right" << std::endl;
    std::cout << distCoeffsR << std::endl;


    imSize = imageSize;
    init();
}

void CameraRectifier::init()
{
    cv::Mat R1, R2, P1, P2, Q;

    cv::stereoRectify(cameraMatrixL, distCoeffsL,
                      cameraMatrixR, distCoeffsR,
                      imSize, rot, trans, R1, R2, P1, P2, Q,
                      CALIB_ZERO_DISPARITY, 1, imSize, &validRoi[0], &validRoi[1]);

    isVert = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

    cv::initUndistortRectifyMap(cameraMatrixL, distCoeffsL, R1, P1,
                                imSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(cameraMatrixR, distCoeffsR, R2, P2,
                                imSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    cout << R2 << endl;
    cout << P2 << endl;


    if( !isVert )
    {
        sf = 600./MAX(imSize.width, imSize.height);
        w = cvRound(imSize.width*sf);
        h = cvRound(imSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imSize.width, imSize.height);
        w = cvRound(imSize.width*sf);
        h = cvRound(imSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }
}

void CameraRectifier::rectify(const cv::Mat &imageLeft, const cv::Mat &imageRight, cv::Mat& result)
{
    cv::Mat rimg, cimg;
    remap(imageLeft, rimg, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
    cvtColor(rimg, cimg, CV_GRAY2BGR);
    cv::Mat canvasPart = !isVert ? canvas(Rect(0, 0, w, h)) : canvas(Rect(0, 0, w, h));
    resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
    Rect vroi(cvRound(validRoi[0].x*sf), cvRound(validRoi[0].y*sf),
                  cvRound(validRoi[0].width*sf), cvRound(validRoi[0].height*sf));
    rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);

    remap(imageRight, rimg, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
    //cv::imshow("Rectificated", rimg);
    cvtColor(rimg, cimg, CV_GRAY2BGR);
    canvasPart = !isVert ? canvas(Rect(w, 0, w, h)) : canvas(Rect(0, h, w, h));
    resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
    Rect vroi2(cvRound(validRoi[1].x*sf), cvRound(validRoi[1].y*sf),
                  cvRound(validRoi[1].width*sf), cvRound(validRoi[1].height*sf));
    rectangle(canvasPart, vroi2, Scalar(0,0,255), 3, 8);

    if( !isVert )
        for(int j = 0; j < canvas.rows; j += 16 )
            line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
    else
        for(int j = 0; j < canvas.cols; j += 16 )
            line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);

    result = canvas;
}

double CameraRectifier::getBaceLine()
{
    return sqrt(pow(trans.at<double>(0, 0),2) +
                pow(trans.at<double>(1, 0),2) + pow(trans.at<double>(2, 0),2));
}

cv::Mat CameraRectifier::getIntrinsicLeft()
{
    return cameraMatrixL;
}

cv::Mat CameraRectifier::getIntrinsicRight()
{
    return cameraMatrixR;
}

cv::Mat CameraRectifier::getTranslation()
{
    return trans;
}
