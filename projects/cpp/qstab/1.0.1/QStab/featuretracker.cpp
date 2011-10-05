#include "featuretracker.h"

FeatureTracker::FeatureTracker() : max_count(500),
            qlevel(0.01), minDist(10.) {
}

void FeatureTracker::process(cv::Mat &frame, cv::Mat &output) {
    cv::cvtColor(frame, gray, CV_BGR2GRAY);
    frame.copyTo(output);

    detectFeaturePoints();
    points[0].insert(points[0].end(),
                     features.begin(),features.end());

    if(gray_prev.empty())
        gray.copyTo(gray_prev);

    cv::calcOpticalFlowPyrLK(
                    gray_prev, gray,
                    points[0],
                    points[1],
                    status,
                    err);

    calcMotion(frame);
    drawMotion(frame, output);

    std::swap(points[1], points[0]);
    cv::swap(gray_prev, gray);

}

void FeatureTracker::detectFeaturePoints() {
    cv::goodFeaturesToTrack(gray,
                    features,
                    max_count,
                    qlevel,
                    minDist);
}

bool FeatureTracker::addNewPoints(){
    return points[0].size()<=10;
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


void FeatureTracker::calcMotion(cv::Mat &frame) {
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
