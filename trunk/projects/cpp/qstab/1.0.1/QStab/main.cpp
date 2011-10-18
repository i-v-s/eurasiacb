#include "qstab.h"
#include <iostream>


void mytrigger(std::vector<double> lMotion, std::vector<double> cMotion, std::vector<double> rMotion) {
    //std::cout<< lMotion << ":" << cMotion << ":" << rMotion << std::endl;
}

int main(){
    //Yo!
    VideoProcessor processor;
    qstab::FeatureTracker tracker;
    //tracker.setTrigger(mytrigger);
    processor.addInput(0);
    processor.setFrameProcessor(&tracker);
    //processor.setOutput("output.avi", CV_FOURCC('D','I','V','X'));
    processor.displayOutput("Cam1");

    processor.setDelay(1000./processor.getFrameRate());
    processor.run();

    return 1;
}
