#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include <opencv2/video/video.hpp>
#include "videoprocessor.h"

class FeatureTracker: public FrameProcessor
{
    cv::Mat gray;
    cv::Mat gray_prev;
    std::vector<cv::Point2f> points[2];
    std::vector<cv::Point2f> features;
    int max_count;
    double qlevel;
    double minDist;
    double rightMotion;
    double leftMotion;
    double verticalMotion;
    std::vector<uchar> status;
    std::vector<float> err;

public:
    FeatureTracker();
    void process(cv:: Mat &frame, cv:: Mat &output);
    void detectFeaturePoints();
    bool addNewPoints();
    void calcMotion(cv::Mat& frame);
    void drawMotion(cv:: Mat &frame,
                    cv:: Mat &output);

};

#endif // FEATURETRACKER_H
