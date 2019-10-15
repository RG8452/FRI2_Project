#include <ros/ros.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>

class Storage {
public:
    Storage() : __index(0) {}
    void add(openpose_ros_msgs::OpenPoseHumanList);
    openpose_ros_msgs::OpenPoseHuman get();
private:
    openpose_ros_msgs::OpenPoseHuman __data[10];
    uint8_t __index;
};
