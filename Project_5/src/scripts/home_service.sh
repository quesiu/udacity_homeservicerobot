#!/bin/sh
xterm  -e  " source /devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find add_markers)/../map/map.world" &
sleep 5
xterm  -e  " source /devel/setup.bash; roslaunch pick_objects amcl_custom.launch map_file:=$(rospack find add_markers)/../map/map.yaml" & 
sleep 5
xterm  -e  " source devel/setup.bash; rosrun add_markers add_markers" &
sleep 5
xterm  -e  " source devel/setup.bash; roslaunch add_markers view_navigation.launch" &
sleep 5
xterm  -e  " source devel/setup.bash; rosrun pick_objects pick_objects"
sleep 10
