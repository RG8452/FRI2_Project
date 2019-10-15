#include <ros/ros.h>
#include <fri2/Storage.h>

void Storage::add(openpose_ros_msgs::OpenPoseHumanList list) {
    ROS_INFO("TEST");
    if(list.num_humans != 0) {
        __index = (__index + 1) % 10;
        __data[__index] = list.human_list[0];
        ROS_INFO("Set index %d", __index);
    }
}

openpose_ros_msgs::OpenPoseHuman Storage::get() {
    ROS_INFO("Get index %d", __index);
    return __data[__index];
}
