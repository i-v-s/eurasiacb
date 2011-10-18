#include "qstab.h"

using namespace qstab;

FeatureTracker::FeatureTracker() : max_count(500),
            qlevel(0.01), minDist(10.) {
    trigger = 0;

    waitForFix = true;
    fixed = false;
}

void FeatureTracker::process(std::vector<cv::Mat> &frames, std::vector<cv::Mat> &outputs, bool isShow) {

    if(isShow) {
        outputs.clear();
        for(std::vector<cv::Mat>::const_iterator it = frames.begin();
                                it != frames.end(); it++) {
            outputs.push_back(it);
        }
    }

    if(waitForFix) {
        gray_prev.clear();
        leftMotion.clear(0.0);
        rightMotion.clear(0.0);
        verticalMotion.clear(0.0);

        for(std::vector<cv::Mat>::const_iterator it = frames.begin();
                                it != frames.end(); it++) {
            cv::Mat gpr;
            cv::cvtColor(it, gpr, CV_BGR2GRAY);
            gray_prev.push_back(gpr);
            detectFeaturePoints();
            leftMotion.push_back(0.0);
            rightMotion.push_back(0.0);
            verticalMotion.push_back(0.0);
        }

        waitForFix = false;
        fixed = true;
    }

    if(fixed) {
        int i = 0;
        for(std::vector<cv::Mat>::const_iterator it = frames.begin(), i;
                                it != frames.end(); it++, i++) {
            cv::cvtColor(it, gray, CV_BGR2GRAY);

            cv::calcOpticalFlowPyrLK(
                        gray, gray_prev[i],
                            points[i][1],
                            points[i][0],
                            status,
                            err);

            calcMotion(i);
        }

        if(trigger) {
            trigger(leftMotion, verticalMotion, rightMotion);
        }

        if(isShow) {
            drawMotion(frames, outputs);
        }
    }

}

void FeatureTracker::detectFeaturePoints() {
    vector<std::vector<cv::Point2f> pt[2];
    cv::goodFeaturesToTrack(gray_prev[gray_prev.size()-1],
                            pt[1],
                            max_count,
                            qlevel,
                            minDist);
    points.push_back(pt);
}

void FeatureTracker::drawMotion(std::vector<cv::Mat> &outputs) {

    for(int i = 0; i < outputs.size(); i++) {
        //LEFT
        cv::circle(outputs[i], cv::Point(outputs[i].cols/3, outputs[i].rows/2),
                   3, cv::Scalar(0,255,0), 3);
        cv::line(outputs[i], cv::Point(outputs[i].cols/3, outputs[i].rows/2),
                 cv::Point(outputs[i].cols/3 + leftMotion[i],outputs[i].rows/2), cv::Scalar(0, 255, 0), 3);

        //CENTER
        cv::circle(output, cv::Point(outputs[i].cols/2, outputs[i].rows/2),
                   3, cv::Scalar(0,255,0), 3);
        cv::line(output, cv::Point(outputs[i].cols/2, outputs[i].rows/2),
                 cv::Point(outputs[i].cols/2, outputs[i].rows/2 + verticalMotion[i]), cv::Scalar(0, 255, 0), 3);


        //RIGHT
        cv::circle(output, cv::Point(2*outputs[i].cols/3, outputs[i].rows/2),
                   3, cv::Scalar(0,255,0), 3);
        cv::line(output, cv::Point(2*outputs[i].cols/3, outputs[i].rows/2),
                 cv::Point(2*outputs[i].cols/3 + rightMotion[i],outputs[i].rows/2), cv::Scalar(0, 255, 0), 3);
    }
}


void FeatureTracker::calcMotion(int num) {
    leftMotion[num] = 0.0;
    rightMotion[num] = 0.0;
    verticalMotion[num] = 0.0;
    int rnum = 0;
    int lnum = 0;
    int cnum = 0;

    int lborder = gray_prev[num].cols/3;
    int rborder = lborder*2;

    for( int i= 0; i < points[1].size(); i++ ) {
        if(status[i]){
            if(points[0][i].x<lborder) {
                leftMotion[num] += points[num][0][i].x-points[num][1][i].x;
                lnum++;
            } else if (points[0][i].x>rborder) {
                rightMotion[num] += points[num][0][i].x-points[num][1][i].x;
                rnum++;
            } else {
                verticalMotion[num] += points[num][0][i].y-points[num][1][i].y;
                cnum++;
            }
        }
    }

    if(lnum!=0) {
        leftMotion[num]/=lnum;
    }
    if(rnum!=0) {
        rightMotion[num]/=rnum;
    }
    if(cnum!=0) {
        verticalMotion[num]/=cnum;
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
