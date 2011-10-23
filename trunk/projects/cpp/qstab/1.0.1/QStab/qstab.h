#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include <opencv2/video/video.hpp>
#include "videoprocessor.h"
#include <sys/timeb.h>

namespace qstab {

class FeatureTracker: public FrameProcessor
{
    cv::Mat gray;
    cv::Mat gray_prev;
    std::vector<cv::Point2f> points[2];
    int max_count;
    double qlevel;
    double minDist;
    double rightMotion;
    double leftMotion;
    double verticalMotion;
    std::vector<uchar> status;
    std::vector<float> err;

    // FOR TEST
    int step;

    bool waitForFix;
    bool fixed;

    void (*trigger)(double, double, double);

    void detectFeaturePoints();
    void calcMotion();
    void drawMotion(cv:: Mat &frame,
                    cv:: Mat &output);
public:
    FeatureTracker();
    void process(cv:: Mat &frame, cv:: Mat &output, bool isShow);
    void setTrigger(void (*triggerFunc) (double, double, double));
    void fix();
    void unfix();
};

}
#endif // FEATURETRACKER_H
