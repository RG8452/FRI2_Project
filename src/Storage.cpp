#include <ros/ros.h>
#include <fri2/Storage.h>

void Storage::add(openpose_ros_msgs::OpenPoseHumanList list) {
    if(list.num_humans != 0) {
	__index = (__index + 1) % 10; __data[__index] = list.human_list[0];
    }
}

void Storage::calibrate(const sensor_msgs::CameraInfo& msg) {
	fx = 1 / msg.K[0];	
	fy = 1 / msg.K[4];
	cx = msg.K[2];
	cy = msg.K[5];
}

void Storage::storePoints(sensor_msgs::PointCloud2Ptr thePoints) {   
    if (__points) delete[] __points;
    theData = thePoints->data.data();
    sensor_msgs::PointCloud output;
    sensor_msgs::convertPointCloud2ToPointCloud(*thePoints.get(), output);
    uint32_t len = output.points.size();
    this->__numPoints = len;
    __points = new geometry_msgs::Point32[len];
    std::memcpy(__points, output.points.data(), len);
}

openpose_ros_msgs::OpenPoseHuman Storage::get() {
    return __data[__index];
}

void Storage::copyPoints(geometry_msgs::Point32 *&buffer) {
    if (!__points) return;
    ROS_INFO("Copying points");
    if (!buffer) buffer = new geometry_msgs::Point32[this->__numPoints];
    std::memcpy(buffer, this->__points, this->__numPoints);
}
