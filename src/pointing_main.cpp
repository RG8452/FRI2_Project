#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/PointWithProb.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include "fri2/ImageProcessor.h"
#include "fri2/Storage.h"
#include "fri2/MakeLine.h"
#include <set>

static darknet_ros_msgs::BoundingBox *g_boxes = nullptr;
static int g_numBoxes = -1;

void store_boxes(const darknet_ros_msgs::BoundingBoxes &boxes) {
	if (g_numBoxes == -1) return; //Can't store data without this value
	if (g_boxes) delete[] g_boxes;
	g_boxes = new darknet_ros_msgs::BoundingBox[g_numBoxes];
	for (auto i = 0; i < g_numBoxes; i++)
		g_boxes[i] = boxes.bounding_boxes.at(i);
}
void store_box_count(const darknet_ros_msgs::ObjectCount &msg) {
	g_numBoxes = msg.count;
}

typedef struct {
    int x;
    int y;
} pixel2d;
//bool insideBoundingBox(darknet_ros_msgs::BoundingBox*, int, pixel2d, int);
bool insideBoundingBox(pixel2d, int);

// Round to the nearest float
inline int nearest(float f) {
    if ((f - 0.5) > (int) f) return ((int) f + 1);
    return (int) f;
}

// Super dump implementation of a bresenham line; checks along the line given
// Will look up and down a certain number of pixels away from the given points
// Note: This algorithm is similar, but not identical to the one on wikipedia
void bressLine(pixel2d start, pixel2d end) {
    float dX = end.x - start.x;
    float dY = end.y - start.y;
    float m = dY/dX;
	int const verticalRange = 2; //Check 2 up, 2 down from calculated pixel
    for (int tx = 0; tx < abs(end.x - start.x); tx++) {
		// Iterating x from the start x to the end x
		pixel2d p { tx + start.x, nearest(m * tx + start.y)};
		if (end.x < start.x) p.x = (start.x - tx); // Account for pointing left
		if (p.x < 0 || p.x > 640 || p.y < 0 || p.y > 480) continue;
		insideBoundingBox(p, verticalRange);
    }
}

/*
void bresenham3D() {

}
*/

// Check if the pixel is inside bounding box
//bool insideBoundingBox(darknet_ros_msgs::BoundingBox* bounding_boxes, int numBoxes, pixel2d pixel, int delta) {
bool insideBoundingBox(pixel2d pixel, int delta) {
	std::set<int> yPts;
	// Gather all x and y coordinates around the point
	yPts.insert(pixel.y);
	for (int i=0; i<delta; i++) {
		yPts.insert(pixel.y + i);
		yPts.insert(pixel.y - i);
	}
	// Iterate through the bounding boxes
	bool xValid = false;
	for (int i = 0; i < g_numBoxes; i++) {
		// Check if there are any valid x values
		/*
		std::vector<int>::iterator ptr = xPts.begin();
		while(ptr != xPts.end()) {
			if (*ptr >= bounding_boxes[i].xmin &&
			    *ptr <= bounding_boxes[i].xmax){
				xValid = true;
			}
			ptr++;
		}*/
		// Check if there are any valid y values
		auto ptr = yPts.begin();
		while(ptr != yPts.end()) { //Could substitute for loop here, but no real point
			if (*ptr >= g_boxes[i].ymin &&
				*ptr <= g_boxes[i].ymax &&
				pixel.x >= g_boxes[i].xmin &&
				pixel.x <= g_boxes[i].xmax) {
				ROS_INFO("x: %d, y: %d; Item ID: %s", pixel.x, *ptr, g_boxes[i].Class.c_str());
				return true;
			}
			ptr++;
		}
	}
	
	return false;
}
 
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

	ros::Subscriber boxes_counter = n.subscribe("/darknet_ros/found_object", 10, &store_box_count);
	ros::Subscriber boxes = n.subscribe("/darknet_ros/bounding_boxes", 10, &store_boxes);


    //Read in the data of humans and points
    openpose_ros_msgs::OpenPoseHuman h;
    while (!mat_read || (!g_boxes) || g_numBoxes == -1) {
		ros::Duration(0.5).sleep();
		ros::spinOnce();
    }
	h = s.get();

	ROS_INFO("Found yolo stuff, num boxes: %d", g_numBoxes);
	for (int i = 0; i < g_numBoxes; i++)
		ROS_INFO("Box %d: class %s, prob %f, x: [%ld,%ld] y: [%ld,%ld]", i, g_boxes[i].Class.c_str(), g_boxes[i].probability,
		         g_boxes[i].xmin, g_boxes[i].xmax, g_boxes[i].ymin, g_boxes[i].ymax);

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

    //MakeLine ml(&n);	
    ros::Rate loop_rate(10);
    int count = 0;
    
	geometry_msgs::Point wristMsg;	
	geometry_msgs::Point elbowMsg;
	//uncomment when the logic to determine which arm we are sending is done to test
	wristMsg.x = armPoints[3];
	wristMsg.y = armPoints[4];
	wristMsg.z = armPoints[5];
	ROS_INFO("Wrist: x %f y %f z %f\n", wristMsg.x, wristMsg.y, wristMsg.z);
	elbowMsg.x = armPoints[0];
	elbowMsg.y = armPoints[1];
	elbowMsg.z = armPoints[2];
	// ml.pubPoints(elbowMsg, wristMsg);

	float slope = (h.body_key_points_with_prob[4].y - h.body_key_points_with_prob[3].y) /
	              (h.body_key_points_with_prob[4].x - h.body_key_points_with_prob[3].x);
	float posY = h.body_key_points_with_prob[4].y;
	int posX = (int) h.body_key_points_with_prob[4].x;
	int startX = posX;
	int startY = (int) posY;

    if (h.body_key_points_with_prob[4].x - h.body_key_points_with_prob[3].x < 0) {
	    ROS_INFO("Pointing left, x: %d, y: %d", posX, (int)posY);
	    //Pointing left
	    for (; posX > 0; posX--) {
			posY -= slope;
			if (posY < 0 || posY > 480) break;
	    }
	    if (posY < 0) posY = 0;
	    else if (posY > 480) posY = 480;
	    else posX = 0;
	    ROS_INFO("After adjustment: x: %d, y: %d", posX, (int) posY);

		ROS_INFO("Running Bresenham Algorithm");
		pixel2d sPixel { startX, startY };
		pixel2d ePixel { posX, (int) posY };
		bressLine(sPixel, ePixel);

	    geometry_msgs::Point end;
	    float posZ = g_matrix.at<float>((int)posY, posX);
	    end.z = posZ;
	    
	    end.x = (posX + .5 - s.cx) * s.fx * posZ;
	    end.y = (posY + .5 - s.cy) * s.fy * posZ;
	    ROS_INFO("X: %f, Y: %f, Z: %f", end.x, end.y, end.z);
	    //ml.pubPoints(end, wristMsg);
	}
	

	/*
	pt2 p1;
	pt2 p2;
	std::vector<pt2> v;
	bressLine(c, &p1, &p2);
	*/

 return 0;
}
