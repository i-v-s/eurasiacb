#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include <opencv2/video/video.hpp>
#include "videoprocessor.h"
#include <sys/timeb.h>

namespace qstab {

class FeatureTracker: public FrameProcessor
{
    cv::Mat gray;
    std::vector<cv::Mat> gray_prev;
    std::vector<std::vector<cv::Point2f> > points;
    int max_count;
    bool fixed;
    bool waitForFix;
    double qlevel;
    double minDist;
    std::vector<double> rightMotion;
    std::vector<double> leftMotion;
    std::vector<double> verticalMotion;
    std::vector<uchar> status;
    std::vector<float> err;

    void (*trigger)(std::vector<double>, std::vector<double>, std::vector<double>);

    void detectFeaturePoints();
    void calcMotion(int num);
    void drawMotion(std::vector<cv::Mat> &outputs);
public:
    FeatureTracker();
    void process(std::vector<cv::Mat> &frames, std::vector<cv::Mat> &outputs, bool isShow);
    void setTrigger(void (*triggerFunc) (std::vector<double>, std::vector<double>, std::vector<double>));
    void fix();
    void unfix();
};

}
#endif // FEATURETRACKER_H
