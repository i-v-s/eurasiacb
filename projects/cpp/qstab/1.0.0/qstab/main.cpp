#include "featuretracker.h"

int main(){

    VideoProcessor processor;
    FeatureTracker tracker;
    processor.setInput(CV_CAP_ANY);
    processor.setFrameProcessor(&tracker);
    processor.displayOutput("Tracked Features");
    processor.setDelay(1000./processor.getFrameRate());
    processor.run();

    return 1;
}
