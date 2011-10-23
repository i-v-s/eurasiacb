#include "qstab.h"

using namespace qstab;

FeatureTracker::FeatureTracker() : max_count(500),
            qlevel(0.01), minDist(10.) {
    trigger = 0;
    fixed = false;
    waitForFix = false;

    step = 0;
}

void FeatureTracker::process(cv::Mat &frame, cv::Mat &output, bool isShow) {

    if(isShow) {
        frame.copyTo(output);
    }

    if(step<100) {
        step++;
        return;
    }

    if(waitForFix) {
        cv::cvtColor(frame, gray_prev, CV_BGR2GRAY);
        detectFeaturePoints();
        waitForFix = false;
        fixed = true;
    }

    if(fixed) {
        cv::cvtColor(frame, gray, CV_BGR2GRAY);
        cv::calcOpticalFlowPyrLK(
                        gray, gray_prev,
                        points[0],
                        points[1],
                        status,
                        err);

        calcMotion();

        if(isShow) {
            drawMotion(frame, output);
        }

    }

    if(trigger) {
        trigger(leftMotion, verticalMotion, rightMotion);
    }

}

void FeatureTracker::detectFeaturePoints() {
    cv::goodFeaturesToTrack(gray_prev,
                    points[0],
                    max_count,
                    qlevel,
                    minDist);
}

void FeatureTracker::drawMotion(cv::Mat &frame, cv::Mat &output) {

    //LEFT
    cv::circle(output, cv::Point(output.cols/3, output.rows/2),
               3, cv::Scalar(0,255,0), 3);
    cv::line(output, cv::Point(output.cols/3, output.rows/2),
             cv::Point(output.cols/3 + leftMotion,output.rows/2), cv::Scalar(0, 255, 0), 3);

    //CENTER
    cv::circle(output, cv::Point(output.cols/2, output.rows/2),
               3, cv::Scalar(0,255,0), 3);
    cv::line(output, cv::Point(output.cols/2, output.rows/2),
             cv::Point(output.cols/2, output.rows/2 + verticalMotion), cv::Scalar(0, 255, 0), 3);


    //RIGHT
    cv::circle(output, cv::Point(2*output.cols/3, output.rows/2),
               3, cv::Scalar(0,255,0), 3);
    cv::line(output, cv::Point(2*output.cols/3, output.rows/2),
             cv::Point(2*output.cols/3 + rightMotion,output.rows/2), cv::Scalar(0, 255, 0), 3);

}


void FeatureTracker::calcMotion() {
    leftMotion = 0.0;
    rightMotion = 0.0;
    verticalMotion = 0.0;
    int rnum = 0;
    int lnum = 0;
    int cnum = 0;

    int lborder = gray.cols/3;
    int rborder = lborder*2;

    for( int i= 0; i < points[1].size(); i++ ) {
        if(status[i]){
            if(points[0][i].x<lborder) {
                leftMotion += points[0][i].x-points[1][i].x;
                lnum++;
            } else if (points[0][i].x>rborder) {
                rightMotion += points[0][i].x-points[1][i].x;
                rnum++;
            } else {
                verticalMotion += points[0][i].y-points[1][i].y;
                cnum++;
            }
        }
    }

    if(lnum!=0) {
        leftMotion/=lnum;
    }
    if(rnum!=0) {
        rightMotion/=rnum;
    }
    if(cnum!=0) {
        verticalMotion/=cnum;
    }

    return;
}

void qstab::FeatureTracker::setTrigger(void (*triggerFunc)(double,double, double))
{
    trigger= triggerFunc;
}

void qstab::FeatureTracker::fix()
{
    waitForFix = true;
}

void qstab::FeatureTracker::unfix()
{
    waitForFix = false;
    fixed = false;
}
