#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <complex>
#include <math.h>

//location and thresholds
float pick_up_location[3] = {3.0, 5.0, 1.0};
float drop_off_location[3] = {-1.0, 0.0, 1.0};
float threshold[2] = {0.3, 0.01};

//robot state flags
bool on_pickup_location = false;
bool picked_up = false;
bool on_dropoff_location = false;
bool delivered = false;

// simple state machine to update robot states, 
// based on pose and pick/drop locations
void update_robot_state(const nav_msgs::Odometry::ConstPtr &msg)
{
  float robot_pos_x = msg->pose.pose.position.x;
  float robot_pos_y = msg->pose.pose.position.y;
  float robot_orientation = msg->pose.pose.orientation.w;

  //ROS_INFO("robot (x=%f, y=%f, w=%f",robot_pos_x, robot_pos_y, robot_orientation);
  // euclidean_dist = pow( pow((pick_up_location[0] - robot_pos_x),2) + pow((pick_up_location[1] - robot_pos_y),2), 0.5);
  // pick-up actions
  if (   std::abs(pick_up_location[0] - robot_pos_x) < threshold[0] 
      && std::abs(pick_up_location[1] - robot_pos_y) < threshold[0] 
      && std::abs(pick_up_location[2] - robot_orientation) < threshold[1])
  {
    if (!on_pickup_location)
    {
      on_pickup_location = true;
    }
  }
  else
  {
    on_pickup_location = false;
  }
  
  // drop-off actions
  if (   std::abs(drop_off_location[0] - robot_pos_x) < threshold[0] 
      && std::abs(drop_off_location[1] - robot_pos_y) < threshold[0] 
      && std::abs(drop_off_location[2] - robot_orientation) < threshold[1])
  {
    if (!on_dropoff_location)
    {
      on_dropoff_location = true;
    }
  }
  else
  {
    on_dropoff_location = false;
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, update_robot_state);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

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

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pick_up_location[0];
    marker.pose.position.y = pick_up_location[1];
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = pick_up_location[2];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

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

    ROS_INFO("Now showing the cube..");
    marker_pub.publish(marker);

    //going towards pickup location
    while (!on_pickup_location)
    {
      ros::spinOnce();
    }

    if (on_pickup_location && !picked_up)
    {
      // remove marker
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      ROS_INFO("Got the package!");
      picked_up = true;
    }

    //going towards drop-off location
    while (!on_dropoff_location)
    {
      ros::spinOnce();
    }

    if (on_dropoff_location && !delivered)
    {
      marker.pose.position.x = drop_off_location[0];
      marker.pose.position.y = drop_off_location[1];
      marker.pose.orientation.w = drop_off_location[2];
      // display marker again
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
      ROS_INFO("Package delivered!");
      delivered = true;
      ros::Duration(10.0).sleep();
    }
    return 0;
  }
}