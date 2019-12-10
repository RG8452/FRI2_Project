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
#include <Eigen/Dense>

//TYPE DEFINITIONS
typedef struct {
	int x;
    int y;
} pixel2d;

//HACKY STUPID GLOBAL VARIABLES THAT SHOULDN"T EXIST
static darknet_ros_msgs::BoundingBox *g_boxes = nullptr;
static int g_numBoxes = -1;
static cv::Mat g_matrix;
static bool mat_read = false;

// forward declarations because we don't have a header
void mapDepthToMatrix(Eigen::MatrixXf &, Storage &);
void bressLine(pixel2d, pixel2d);
bool insideBoundingBox(pixel2d, int);

//methods that ryan didn't feel like moving
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

// Round to the nearest float
inline int nearest(float f) {
    if ((f - 0.5) > (int) f) return ((int) f + 1);
    return (int) f;
}

void depth_test_cb(const sensor_msgs::Image img) {
    g_matrix = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    mat_read = true; 
}

// actually relevant
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
    ros::Subscriber boxes_counter = n.subscribe("/darknet_ros/found_object", 10, &store_box_count);
    ros::Subscriber boxes = n.subscribe("/darknet_ros/bounding_boxes", 10, &store_boxes);

    //Read in the data of humans and points
    openpose_ros_msgs::OpenPoseHuman h;
    while (!mat_read || (!g_boxes) || g_numBoxes == -1) {
		ros::Duration(0.5).sleep();
		ros::spinOnce();
    }
	h = s.get();

	for (int i = 0; i < g_numBoxes; i++)
		ROS_INFO("Box %d: class %s, prob %f, x: [%ld,%ld] y: [%ld,%ld]", i,
				  g_boxes[i].Class.c_str(), g_boxes[i].probability,
		          g_boxes[i].xmin, g_boxes[i].xmax, g_boxes[i].ymin, g_boxes[i].ymax);

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

	// All bresenham line calculations
	float slope = (h.body_key_points_with_prob[4].y - h.body_key_points_with_prob[3].y) /
	              (h.body_key_points_with_prob[4].x - h.body_key_points_with_prob[3].x);
	float posY = h.body_key_points_with_prob[4].y;
	int posX = (int) h.body_key_points_with_prob[4].x;
	int startX = posX;
	int startY = (int) posY;

    Eigen::MatrixXf pointMatrix;
	// If we are pointing left (from the camera's perspective)
    if (h.body_key_points_with_prob[4].x - h.body_key_points_with_prob[3].x < 0) {
	    ROS_INFO("Pointing left, x: %d, y: %d", posX, (int)posY);
	    //Pointing left, calculate endpoint of line (where it leaves the screen)
	    for (; posX > 0; posX--) {
			posY -= slope;
			if (posY < 0 || posY > 480) break;
	    }
	    if (posY < 0) posY = 0;
	    else if (posY > 480) posY = 480;
	    else posX = 0;
	    ROS_INFO("After adjustment: x: %d, y: %d", posX, (int) posY); //Found endpoint of bresenham

		ROS_INFO("Running Bresenham Algorithm");
		pixel2d sPixel { startX, startY };
		pixel2d ePixel { posX, (int) posY };
		bressLine(sPixel, ePixel);

	    mapDepthToMatrix(pointMatrix, s); // Fill out point matrix
		// At this point, the pointMatrix has the following format:
		// [x0, y0, z0]
		//  ...
		// [x307199 y307199 z307199]
		// Each row represents the calculated spatial coordinates of each pixel
		// The pixels (per row) are represented in turn as row-major order
	}

	// We want a 4 row, 480 * 640 column matrix that holds the 3D points from the depth camera
	// We multiply the 3 x 4 projective matrix by this matrix to get the camera points
	// Then, camera point (x, y) is the output matrix 640 * y + x / w
	// The resulting w represents the depth, and we always want the smaller w because it is
	// closer to the camera

    // Formulating pMat 
    Eigen::MatrixXf rMat(3, 4); // Rigid Transformation Matrix (ignoring fourth row); 3 x 4
								// Notably, this is also the Pideal matrix
    rMat << 1, 0, 0, -0.025,
            0, 1, 0, 0.0,
            0, 0, 1, 0.0;

    Eigen::MatrixXf kMat(3, 3); // Constant matrix for adjusting values; 3 x 3
								// Should describe the color camera; TODO: verify
    kMat << 525.0, 0.0,   319.5,
          	0.0,   525.0, 239.0,
            0.0,   0.0,   1.0;

	// Projective matrix: Formed from psuedoidentity rotation, plus translation, multiplied by constants
	// This is the real projective matrix, not the ideal projective matrix, and is 3 x 4
	// I *believe* that this matrix projects points from the RGB camera into the depth frame
    Eigen::MatrixXf pMat = kMat * rMat;

	// We cannot invert the pMat this way; instead, we would have to invert the rigid transformation matrix
	// and calculate a new one.
    // pMat.inverse();

	// Create a huge 2D array that represents the mapping for the depth of each pixel in the color camera
	// Notably, not every single function will be defined for this operation
    Eigen::MatrixXd map(480,640);

	/*
    for (int i = 0; i < 480 * 640; i++) {
		Eigen::MatrixXf row = pointMatrix.block(i, 0, 1, 3);
		row.transpose();
		Eigen::MatrixXf ret = row * pMat;
    }*/
	for (int r = 0; r < 480; r++) {
		Eigen::MatrixXf row = pointMatrix.block(row, 0, 1, 3); // Get one row from the point matrix
		Eigen::MatrixXf tRow(4, 1); // Create transpose row with psuedo-homography coordinate, 4 x 1
		tRow(0, 0) = row(0, 0);  //				[x]
		tRow(1, 0) = row(0, 1);  //				[y]
		tRow(2, 0) = row(0, 2);  //				[z]
		tRow(3, 0) = 1.0;        // Fake value  [1]
		Eigen::MatrixXf ret = pMat * row; // Returned point, 4 x 1
	}
    return 0;
}

/* 
 * Calculate the x,y,z coordinates of a pixel from the depth camera perspective
 * Uses the g_matrix of depth coordinates and image registration math to determine
 * where the point is, stores result in float references
 */
void calculateCoords(Storage &s, int index, float &x, float &y, float &z) {
    int row = index / 640;
    int col = index % 640;
    z = g_matrix.at<float>(row, col);
    x = (col + 0.5 - s.cx) * s.fx * z;
    y = (row + 0.5 - s.cy) * s.fy * z;
}

/*
 * Store all of the depth points into a big Eigen matrix
 * Resulting matrix shall have 480 * 640 rows, each with 3 columns
 * The format of the columns is [x, y, z]
 * Requires a Storage object that has the camera constants
 */
void mapDepthToMatrix(Eigen::MatrixXf &pMatrix, Storage &s) {
    for (int index = 0; index < 480 * 640; index++) {
		float x, y, z;
		calculateCoords(s, index, x, y, z);
		pMatrix(index, 1) = x;
		pMatrix(index, 2) = y;
		pMatrix(index, 3) = z;
    }
}

// Super dumb implementation of a bresenham line; checks along the line given
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

// Check if the pixel is inside bounding box
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
