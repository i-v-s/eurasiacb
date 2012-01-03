#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../../../cpp/qstab/1.0.1/QStab/qstab.h"
#include "../../../cpp/qstab/1.0.1/QStab/videoprocessor.h"

ros::Publisher cameraData_pub;

void mytrigger(double lMotion, double cMotion, double rMotion) {
    ROS_INFO("l: %f, c: %f", lMotion, cMotion);

    geometry_msgs::Twist msg;
    msg.linear.x = lMotion;
    msg.linear.y = cMotion;
    cameraData_pub.publish(msg);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "stabtalker");
    ros::NodeHandle n;
    cameraData_pub = n.advertise<geometry_msgs::Twist>("cameraData", 1000);

    VideoProcessor processor;
    qstab::FeatureTracker tracker;
    tracker.fix();
    tracker.setTrigger(mytrigger);
    processor.setInput(CV_CAP_ANY);
    processor.setFrameProcessor(&tracker);

    processor.setDelay(1000./processor.getFrameRate());
    ros::Rate loop_rate(10);

    if(processor.isOpened())
        while(ros::ok()) {
            if(!processor.runOnce()) break;
            ros::spinOnce();
            loop_rate.sleep();
        }


    return 1;
}
