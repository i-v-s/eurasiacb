#include "ros/ros.h"
//#include "qstab/Move.h"
//#include "qstab.h"

#include <sstream>

ros::Publisher chatter_pub;

void trigger(double lMotion, double cMotion, double rMotion) {
    ROS_INFO("%ld", 5);
/*
    msg.horizontal = 10;
    msg.vertical = 20;

    chatter_pub.publish(msg);

*/
}


int main(int argc, char **argv)
{
/*
	ros::init(argc, argv, "stabtalker");
	ros::NodeHandle n;
	chatter_pub = n.advertise<qstab::Move>("chatter", 1000);

	qstab::Move msg;

	VideoProcessor processor;
	qstab::FeatureTracker tracker;
	tracker.setTrigger(trigger);
	processor.setInput(CV_CAP_ANY);
	processor.setFrameProcessor(&tracker);
	processor.setDelay(1000./processor.getFrameRate());
	processor.run();

*/
	return 0;
}
