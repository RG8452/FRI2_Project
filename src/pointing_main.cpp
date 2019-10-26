#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/PointWithProb.h>

#include "fri2/ImageProcessor.h"
#include "fri2/Storage.h"
 
static cv::Mat g_matrix;
static bool mat_read = false;

void depth_test_cb(const sensor_msgs::Image img) {
    g_matrix = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    mat_read = true; 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hw5_node");
    ros::NodeHandle n;
    Storage s;

    //Human subscription
    ros::Subscriber openpose_sub = n.subscribe<openpose_ros_msgs::OpenPoseHumanList>(
		                              "/openpose_ros/human_list", 10, &Storage::add, &s); 
    //Calibrate subscription
    ros::Subscriber depth_camera_info = n.subscribe("/camera/depth/camera_info", 1, &Storage::calibrate, &s);
    
    //Image (points) subscription
    ros::Subscriber depth_image_test = n.subscribe("/camera/depth/image", 10, &depth_test_cb);

    //Read in the data of humans and points
    openpose_ros_msgs::OpenPoseHuman h;
    while (!mat_read) {
	ros::Duration(0.5).sleep();
	ros::spinOnce();
	h = s.get();
    }

    //Debug out everythin
    ROS_INFO("Right arm: 3, right wrist: 4, left arm: 6, left wrist: 7");
    uint32_t points[4] {3, 4, 6, 7};
    for(int i = 0; i < 4; i++) {
	uint32_t j = points[i];
	float curZ = g_matrix.at<float>((int) h.body_key_points_with_prob[j].y, (int) h.body_key_points_with_prob[j].x);
	float curX = (h.body_key_points_with_prob[j].x + .5 - s.cx) * s.fx * curZ;
	float curY = (h.body_key_points_with_prob[j].y + .5 - s.cy) * s.fy * curZ;
	ROS_INFO("Point %d\t x: %f, y: %f, z: %f", j, curX, curY, curZ);
    }

 return 0;
}
