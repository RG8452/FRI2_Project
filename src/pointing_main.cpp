#include "ros/ros.h"
#include "fri2/test.h"

//Callback function, for sure not written correctly
/*
void receiveImage(const std_msgs::StringConstPtr &msg) {
	ROS_INFO(msg);
}
*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "hw5_node");
    ros::NodeHandle n;

    //ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, receiveImage);

    return 0;
}
