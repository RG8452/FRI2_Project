# FRI2_Project

## About
This project is a research project for the Freshman Research Initiative at the University of Texas at Austin. The goal of this project is ultimately to create a software program that will enable a robot to follow the direction a person is pointing and determine the object that person is pointing at. [Here](https://www.overleaf.com/8178441511hhgghwzctwtj) is a link to edit the Overleaf document for this project's proposal for more information, and [here](https://www.overleaf.com/read/prmwvtgfdbkz) is a link to view it.

## Approach
We hope to achieve this through a simple process:
1. Determine the length of a person's arm using [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)
2. Take a picture of a person pointing using a Kinect
3. Detect where their arm is in space using a combination of technologies: OpenPose is used on the HD RGB image to find the u,v coordinates in the plane of the image, the Kinect provides a depth image to find the rough depth at those points, and a registered image mapping function can translate those points into x, y, and z
4. Create a vector in space which begins at the person's elbow and extends towards their fingers, and scale it into space infinitely
5. Find where that vector first intersects an object in space
6. Run object detection (i.e. [YOLO](https://pjreddie.com/darknet/yolo/)) to determine what the user is pointing at

We would like for the robot to have fairly high prediction accuracy, and in order to compare its performance we want to set up an experiment where the robot and a human participant both try to deduce what a second human participant is pointing at. The accuracy of the two predictions can then be compared.

## Setup
To initialize this repository, you should run the following process:
* `mkdir my_catkin_ws`
* `cd my_catkin_ws`
* `catkin_init_workspace`
* `mkdir src`
* `catkin build`
* `cd src`
* `git clone https://github.com/RG8452/FRI2_Project.git`
* `cd ..`
* `catkin build`

This should create a workspace and clone the existing repository into it.

## Initialization
In order to productively run code, you'll need to have several processes running through terminal. If you open multiple terminals for these processes, each terminal **must** have sourced devel/setup.bash (from this directory). Here is a list of the commands that should be running, in the order that they need to be run:

* `source devel/setup.bash` (Use from the catkin workspace)
* `roslaunch freenect_launch freenect.launch` (This also runs roscore, provided it isn't running already)
  * `roslaunch bwi_launch v2.launch` (I'm told something like this exists for the robot but idk)
* `rosrun rviz rviz`
* `rosrun fri2_pkg <file name>`
* TODO: figure out what all else must be run

All source code implementations should be in the src directory, and all source code interfaces (the .h files) should be put in the include/fri2 directory.

## YOLO
The YOLO package that we intend to use alongside our project is a custom ROS package located in the GitHub repo [leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros). For reference in using/subscribing to the topic that this repo publishes, you can reference Parth's repo [here](https://github.com/ParthChonkar/FRI_FinalProject). Specifically, the file [subscriber.py](https://github.com/ParthChonkar/FRI_FinalProject/blob/master/identification_protocol/src/subscriber.py) might serve as a useful reference.

The LeggedRobotics darknet_ros package must be in the **same workspace src folder** as fri2_pkg in order to run. This must be configured in the CMakeLists file in order to function properly. Furthermore, the path variable in the ImageHandler class (called imgFolderPath) so that its absolute path points to (and includes) the rospackage on your local machine.
