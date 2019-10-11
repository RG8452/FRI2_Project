#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>

//#include "libfreenect.h"
//#include "fri2/test.h"
#include "fri2/ImageProcessor.h"
#include "fri2/Storage.h"

/*
 * There's some weird jank going on here, I'm honestly just testing the kinect
 * ImageProcessor supports string and encoding construction
 * Encodings are std::strings found in image_encodings.h, as seen here:
 * http://docs.ros.org/jade/api/sensor_msgs/html/image__encodings_8h_source.html
 *
 * Honestly this is just testing stuff but I want to see what all the images are
 * Dr Hart claimed something in here should be hi-res, but all I'm finding is 480x640
 */

int main(int argc, char **argv) {
    ros::init(argc, argv, "hw5_node");
    ros::NodeHandle n;

    /*
    ImageProcessor ip("/camera/rgb/image_color");
    ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("/camera/rgb/image_color",
                                                          100, &ImageProcessor::imageCb, &ip);

    ImageProcessor ip2("/camera/rgb/image_rect_color");
    ros::Subscriber sub2 = n.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                          100, &ImageProcessor::imageCb, &ip2);

    ImageProcessor ip3("/camera/depth/image", sensor_msgs::image_encodings::TYPE_32FC1);
    ros::Subscriber sub3 = n.subscribe<sensor_msgs::Image>("/camera/depth/image",
                                                          100, &ImageProcessor::imageCb, &ip3);

    ImageProcessor ip4("/camera/depth/image_rect", sensor_msgs::image_encodings::TYPE_32FC1);
    ros::Subscriber sub4 = n.subscribe<sensor_msgs::Image>("/camera/depth/image_rect",
                                                          100, &ImageProcessor::imageCb, &ip4);
    */

    Storage<openpose_ros_msgs::OpenPoseHumanList> s;
    ros::Subscriber openpose_sub = n.subscribe<openpose_ros_msgs::OpenPoseHumanList>(
		                               "/openpose_ros/human_list", 10, &Storage::add, &s);

    ros::spin();

    return 0;
}
