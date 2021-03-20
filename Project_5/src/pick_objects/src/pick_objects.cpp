#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pick_up;
  move_base_msgs::MoveBaseGoal drop_off;

  // set up the frame parameters
  pick_up.target_pose.header.frame_id = "map";
  pick_up.target_pose.header.stamp = ros::Time::now();
  drop_off.target_pose.header.frame_id = "map";
  drop_off.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach for pick up
  pick_up.target_pose.pose.position.x = 2.11;
  pick_up.target_pose.pose.position.y = -6.96;
  pick_up.target_pose.pose.position.z = 0.0;
  pick_up.target_pose.pose.orientation.w = 1.0;

  // Define a position and orientation for the robot to reach for drop off
  drop_off.target_pose.pose.position.x = 1.81;
  drop_off.target_pose.pose.position.y = 1.0;
  drop_off.target_pose.pose.position.z = 0.0;
  drop_off.target_pose.pose.orientation.w = 1.0;

  // Send the pick up position and orientation for the robot to reach
  ROS_INFO("Sending pick up");
  ac.sendGoal(pick_up);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Pick up zone successfully reached");
  else
    ROS_INFO("Failed to reach pick up zone");

  // Pause for 5 seconds to simulate a pick-up
  ros::Duration(5).sleep();


  // Send the drop off position and orientation for the robot to reach
  ROS_INFO("Sending drop off");
  ac.sendGoal(drop_off);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Drop off zone successfully reached");
  else
    ROS_INFO("Failed to reach drop off zone");

  // Pause for 5 seconds to simulate a pick-up
  ros::Duration(5).sleep();

  return 0;
}
