#include "featuretracker.h"
#include <iostream>

void mytrigger(double lMotion, double cMotion, double rMotion) {
    std::cout<< lMotion << ":" << cMotion << ":" << rMotion << std::endl;
}

int main(){
    //Yo!
    VideoProcessor processor;
    qstab::FeatureTracker tracker;
    //tracker.setTrigger(mytrigger);
    processor.setInput(CV_CAP_ANY);
    processor.setFrameProcessor(&tracker);
    //processor.setOutput("output.avi", CV_FOURCC('D','I','V','X'));
    processor.displayOutput("Tracked Features");

    processor.setDelay(1000./processor.getFrameRate());
    processor.run();

    return 1;
}
