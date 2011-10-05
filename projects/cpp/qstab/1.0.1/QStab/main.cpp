#include "featuretracker.h"

int main(){
    //Yo!
    VideoProcessor processor;
    FeatureTracker tracker;
    processor.setInput(CV_CAP_ANY);
    processor.setFrameProcessor(&tracker);
    processor.setOutput("output.avi", CV_FOURCC('D','I','V','X'));
    processor.displayOutput("Tracked Features");
    processor.setDelay(1000./processor.getFrameRate());
    processor.run();

    return 1;
}
