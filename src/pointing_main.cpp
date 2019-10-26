#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointField.h>
#include <cv_bridge/cv_bridge.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/PointWithProb.h>

//#include "libfreenect.h"
#include "fri2/ImageProcessor.h"
#include "fri2/Storage.h"
#include <visualization_msgs/Marker.h>
/*
 * There's some weird jank going on here, I'm honestly just testing the kinect
 * ImageProcessor supports string and encoding construction
 * Encodings are std::strings found in image_encodings.h, as seen here:
 * http://docs.ros.org/jade/api/sensor_msgs/html/image__encodings_8h_source.html
 *
 * Honestly this is just testing stuff but I want to see what all the images are
 * Dr Hart claimed something in here should be hi-res, but all I'm finding is 480x640
 */
 
 /*
 	I believe that this is what is being returned, in that file if you look there is a field K
 	K from my understanding is a 3x3 matrix that contains fx, fy, cx, cy, which is im assuming what we want, according to the similiar variable names used in the registration.cpp file in freenect2
 	http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
 */

static cv::Mat g_matrix;
static bool mat_read = false;


void depth_test_cb(const sensor_msgs::Image img) {
    ROS_INFO("Height: %d\t Width: %d\t Step: %d", img.height, img.width, img.step);
    ROS_INFO("Encoding: %s", img.encoding.c_str());

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
    
    //ros::Subscriber depth_camera_pointcloud = n.subscribe("/camera/depth/points", 10, &Storage::storePoints, &s);

    ros::Subscriber depth_image_test = n.subscribe("/camera/depth/image", 10, &depth_test_cb);

    //ros::Subscriber raw_depth_camera = n.subscribe("/camera/depth/raw");
    
    /*
    //geometry_msgs::Point32 *buffer = nullptr;
    openpose_ros_msgs::OpenPoseHuman h;
    while (!buffer) {
	ros::Duration(0.5).sleep();
        ros::spinOnce();
	s.copyPoints(buffer);
	h = s.get();
    } 
    
    int index = (int) (640 * h.body_key_points_with_prob[6].y + h.body_key_points_with_prob[6].x);

    ROS_INFO("Using cv::Mat, value: %f", g_matrix.at<float>((int) h.body_key_points_with_prob[6].y, (int) h.body_key_points_with_prob[6].x));

    float testPoint = (buffer[index].z) / 1000.0f;
    
    float sampleTranslatedX = (h.body_key_points_with_prob[6].x + .5 - s.cx) * s.fx * testPoint;
    float sampleTranslatedY = (h.body_key_points_with_prob[6].y + .5 - s.cy) * s.fy * testPoint;
    //ROS_INFO("Raw Data x - RGB: %f, Depth Val %f", h.body_key_points_with
    ROS_INFO("X - RGB Val: %f, Depth Val: %f", h.body_key_points_with_prob[6].x, sampleTranslatedX);
    ROS_INFO("Y - RGB Val: %f, Depth Val: %f", h.body_key_points_with_prob[6].y, sampleTranslatedY);
    ROS_INFO("Calibration Data: cx %f, cy %f, fx %f, fy %f,", s.cx, s.cy, s.fx, s.fy);
    */
//	ros::spin();
    openpose_ros_msgs::OpenPoseHuman h;
    while (!mat_read) {
	ros::Duration(0.5).sleep();
	ros::spinOnce();
	h = s.get();
    }


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
