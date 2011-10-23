#include "ros/ros.h"
#include "std_msgs/Int32.h"

//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "publoc");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("chatter", 1000);

  ros::Rate loop_rate(10);

  //cv::Mat img = cv::imread("delacroix.jpg");

  int count = 0;
  while (ros::ok())
  {
    std_msgs::Int32 msg;

    msg.data = 1; //img.rows;

    ROS_INFO("%d", msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}


