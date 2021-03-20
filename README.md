# udacity_homeservicerobot
Git repository for Project 5 - Home Service Robot of [Udacity Robotics Software Engineer nanodegree](https://www.udacity.com/course/robotics-software-engineer--nd209).

## Description
By using different learnings from all previous projects and by triggering different scrips, this projects aims at gradually simulate a home service delivery robot.

## Content
- add_markers: package to add virtual markers on a rviz vizualisation to simulate pick-up and drop-off
- map: contains maps and world for this project. amcl_map is generated via gmapping but with poor results
- pick_objects: package to add path planning to two set points
- rvizConfig: contains rviz vizualiation preset file with Markers activated for add_markers package
- scripts: scripts to launch all sub-parts of this project
All other packages are submodules from existing git repository. Please consult their ReadMe file directly.

## Pre-requisites
This project requires a Linux machine with Gazebo and ROS (including catkin and RViz), as well as all of their dependencies. 
Git is also recommended to clone this repository.

## Installation
Please follow the procedure below to be able to correctly build the project:
```
git clone --recurse-submodules https://github.com/quesiu/udacity_homeservicerobot.git
cd /<YOUR_LOCAL_PATH>/udacity_homeservicerobot
mv Project_5/ catkin_ws/
cd catkin_ws/
```

## Build
Do the following to build the project:
```
cd /<YOUR_LOCAL_PATH>/udacity_homeservicerobot/catkin_ws
catkin_make
```

## Usage

###launch.sh
This script is a just a first trial to shell scripting.
```
cd /<YOUR_LOCAL_PATH>/udacity_homeservicerobot/catkin_ws
source devel/setup.bash
./src/scripts/launch.sh
```

###test_slam.sh
This script launches all necessary nodes to allow slam via gmapping.
It is very difficult to get good results with this script due to:
- poor sensor detection from turtlebot default robot
- poor AMCL algorithm for Roomba type robots, rotating on themselves
For best performances though, try to turn as little as possible while keeping on having a forward movement. Do several passes both clockwise and anti-clockwise, favorating straight lines.
```
cd /<YOUR_LOCAL_PATH>/udacity_homeservicerobot/catkin_ws
source devel/setup.bash
./src/scripts/test_slam.sh
```
Alternatively test_slam_turtlebot_world.sh can be used with default world files from turtlebot gazebo.

###test_slam.sh
This script launches all necessary nodes to allow slam via gmapping.
With this algorithm and a map generating via SLAM, this allows to give a 2D goal to the robot in the map (via RViz vizualisation).
Robot will use a path planning algorithm to reach its destination.
```
cd /<YOUR_LOCAL_PATH>/udacity_homeservicerobot/catkin_ws
source devel/setup.bash
./src/scripts/test_navigation.sh
```
Alternatively test_navigation_turtlebot_world.sh can be used with default world files from turtlebot gazebo.

###pick_objects.sh
This script launches all necessary nodes to allow the robot to pick items at two given positions.
```
cd /<YOUR_LOCAL_PATH>/udacity_homeservicerobot/catkin_ws
source devel/setup.bash
./src/scripts/pick_objects.sh
```
Alternatively pick_objects_turtlebot_world.sh can be used with default world files from turtlebot gazebo.

###add_markers.sh
This script launches all necessary nodes to allow the addition of virtual packages on the RViz vizualisation, simulating objects the robot could take.
```
cd /<YOUR_LOCAL_PATH>/udacity_homeservicerobot/catkin_ws
source devel/setup.bash
./src/scripts/add_markers.sh
```

###add_markers.sh
This script launches all necessary nodes to allow full home service experience:
- the robot moves towards a defined pick-up point first, grabs an item and delivers it to a defined drop-off point
- previously coded pick_objects node is used to lead the robot to its goals using path planning and the map generating before
- previously coded add_markers node is slightly modified to simulate the delivery: it subscribes to the odometry data from the robot and calculates if being close enough to the different zones to pick-up/drop-off the items.
It is important to note that odometry data and AMCL data (RViz) are not using the same coordinates.
```
cd /<YOUR_LOCAL_PATH>/udacity_homeservicerobot/catkin_ws
source devel/setup.bash
./src/scripts/home_service.sh
```
