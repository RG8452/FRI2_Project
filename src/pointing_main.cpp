#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/PointWithProb.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include "fri2/ImageProcessor.h"
#include "fri2/Storage.h"
#include "fri2/MakeLine.h"
 
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
    ros::Publisher wristPub = n.advertise<geometry_msgs::Point>("elbow", 1000);
    ros::Publisher elbowPub = n.advertise<geometry_msgs::Point>("wristPub", 1000);
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
    }
	h = s.get();
    //while!point picked
    ROS_INFO("Right arm: 3, right wrist: 4, left arm: 6, left wrist: 7");
    uint32_t points[4] {3, 4, 6, 7};
    float armPoints[12]; //array with all the points
    for(int i = 0; i < 4; i++) {
	uint32_t j = points[i];
	float curZ = g_matrix.at<float>((int) h.body_key_points_with_prob[j].y, (int) h.body_key_points_with_prob[j].x);
	armPoints[3 * i + 2] = curZ;
	float curX = (h.body_key_points_with_prob[j].x + .5 - s.cx) * s.fx * curZ;
	armPoints[3 * i] = curX;
	float curY = (h.body_key_points_with_prob[j].y + .5 - s.cy) * s.fy * curZ;
 	armPoints[3 * i + 1] = curY;
    }
    /*
    for(int i = 0; i < 12; i++) {
    	if(armPoints[i] == 0 || isnan(armPoints[i]) {
		ros::spinOnce(); 		
	}
    } */ 	

    MakeLine ml(&n);	
    ros::Rate loop_rate(10);
    int count = 0;
    
	geometry_msgs::Point wristMsg;	
	geometry_msgs::Point elbowMsg;
	//uncomment when the logic to determine which arm we are sending is done to test
	wristMsg.x = armPoints[3];
	wristMsg.y = armPoints[4];
	wristMsg.z = armPoints[5];
	elbowMsg.x = armPoints[0];
	elbowMsg.y = armPoints[1];
	elbowMsg.z = armPoints[2];
	//wristPub.publish(wristMsg);
	//elbowPub.publish(elbowMsg); 
	ml.pubPoints(elbowMsg, wristMsg);
 return 0;
}
