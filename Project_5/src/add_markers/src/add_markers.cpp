#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

float posX = 0;
float posY = 0;
bool pickupZone = false;
bool dropOffZone = false;
// Different coordinate system for both AMCL = Rviz and Odometry = 
// A better solution would be to use the same for all
float pickUpX = 6.878;
float pickUpY = 2.06;
float dropOffX = -0.884;
float dropOffY = 1.628;
float pickUpXAMCL = 2.11;
float pickUpYAMCL = -6.96;
float dropOffXAMCL = 1.81;
float dropOffYAMCL = 1.0;

// Get odometry information and calculate where robot is
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // ROS_INFO("%s", msg->header.frame_id.c_str());
  // ROS_INFO("%f", msg->twist.twist.linear.x);
  ROS_INFO("Robot X pose: %f", msg->pose.pose.position.x);
  ROS_INFO("Robot Y pose: %f", msg->pose.pose.position.y);
  posX = msg->pose.pose.position.x;
  posY = msg->pose.pose.position.y;
  // Calculate if close to pick up and drop off
  float pickUpDistance = sqrt(pow(posX-pickUpX,2)+pow(posY-pickUpY,2));
  float dropOffDistance = sqrt(pow(posX-dropOffX,2)+pow(posY-dropOffY,2));
  ROS_INFO("pickUp distance: %f", pickUpDistance);
  ROS_INFO("dropOff distance: %f", dropOffDistance);
	
  // Close to pick up (arbitrary distance)
  if(pickUpDistance < 0.5)
  {
    ROS_INFO("In pick up zone");
    pickupZone = true;
  }
  // Close to drop off (arbitrary distance)
  if(dropOffDistance < 0.5)
  {
    ROS_INFO("In drop off zone");
    dropOffZone = true;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  //Subscribe to odometry
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, &odomCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  // 0 > pick up, 1 > disappear, 2 > drop off
  int phase = 0; 
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type: CUBE
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker for pick-up zone.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pickUpXAMCL;
    marker.pose.position.y = pickUpYAMCL;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    if(phase == 2)
    {
      // Change to drop off zone
      // Set the pose of the marker for pick-up zone.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = dropOffXAMCL;
      marker.pose.position.y = dropOffYAMCL;
      // ADD new marker at drop off
      marker.action = visualization_msgs::Marker::ADD;
    }
    if(phase == 1)
    {
      // DELETE/hide pick-up marker
      marker.action = visualization_msgs::Marker::DELETE;
      if(dropOffZone)
      {
        phase = 2; // Switch to drop-off
      }
    }
    if(phase == 0)
    {
      // ADD new marker at drop off
      marker.action = visualization_msgs::Marker::ADD;
      if(pickupZone)
      {
        phase = 1; // Switch to disappear = picked-up
      }
    }

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    // Wait 5 sec
    ros::Duration(5).sleep();

    // Cycle between different shapes
    /*switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }*/

    ros::spinOnce();
  }
}
