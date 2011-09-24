#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include <opencv2/video/video.hpp>
#include "videoprocessor.h"

class FeatureTracker: public FrameProcessor
{
    cv::Mat gray;
    cv::Mat gray_prev;
    std::vector<cv::Point2f> points[2];
    std::vector<cv::Point2f> initial;
    std::vector<cv::Point2f> features;
    int max_count;
    double qlevel;
    double minDist;
    std::vector<uchar> status;
    std::vector<float> err;

public:
    FeatureTracker();
    void process(cv:: Mat &frame, cv:: Mat &output);
    void detectFeaturePoints();
    bool addNewPoints();
    bool acceptTrackedPoint(int i);
    void handleTrackedPoints(cv:: Mat &frame,
                    cv:: Mat &output);

};

#endif // FEATURETRACKER_H
