#include "featuretracker.h"

FeatureTracker::FeatureTracker() : max_count(500),
            qlevel(0.01), minDist(10.) {
}

void FeatureTracker::process(cv::Mat &frame, cv::Mat &output) {
    cv::cvtColor(frame, gray, CV_BGR2GRAY);
    frame.copyTo(output);

    // 1. if new feature points must be added
    if(addNewPoints())
    {
        detectFeaturePoints();
        points[0].insert(points[0].end(),
                         features.begin(),features.end());
        initial.insert(initial.end(),
                       features.begin(),features.end());
    }

    if(gray_prev.empty())
        gray.copyTo(gray_prev);

    cv::calcOpticalFlowPyrLK(
                    gray_prev, gray,
                    points[0],
                    points[1],
                    status,
                    err);

    // 2. loop over the tracked points to reject some
    int k=0;
    for( int i= 0; i < points[1].size(); i++ ) {
        if (acceptTrackedPoint(i)) {
            initial[k]= initial[i];
            points[1][k++] = points[1][i];
        }
    }
    // eliminate unsuccesful points
    points[1].resize(k);
    initial.resize(k);

    // 3. handle the accepted tracked points
    handleTrackedPoints(frame, output);

    // 4. current points and image become previous ones
    std::swap(points[1], points[0]);
    cv::swap(gray_prev, gray);

}

void FeatureTracker::detectFeaturePoints() {
    // detect the features
    cv::goodFeaturesToTrack(gray,
                    features,
                    max_count,
                    qlevel,
                    minDist);
}

bool FeatureTracker::addNewPoints(){
    return points[0].size()<=10;
}

bool FeatureTracker::acceptTrackedPoint(int i) {
    return status[i] &&
                    (abs(points[0][i].x-points[1][i].x)+
                    (abs(points[0][i].y-points[1][i].y))>2);

}

void FeatureTracker::handleTrackedPoints(cv::Mat &frame, cv::Mat &output) {
    for(int i= 0; i < points[1].size(); i++ ) {
        cv::line(output,
                        initial[i],
                        points[1][i],
                        cv::Scalar(255,255,255));
        cv::circle(output, points[1][i], 3,
                   cv::Scalar(255,255,255),-1);
    }
}
