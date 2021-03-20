#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; roslaunch  turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  " rosrun rviz rviz" 
