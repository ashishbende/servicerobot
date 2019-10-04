#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//locations
float pick_up_location[3] = {3.0, 5.0, 1.0};
float drop_off_location[3] = {-1.0, 0.0, 1.0};

int main(int argc, char **argv)
{
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pick_up_location[0];
  goal.target_pose.pose.position.y = pick_up_location[1];
  goal.target_pose.pose.orientation.w = pick_up_location[2];

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick-up goal.");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)

  {
    ROS_INFO("Yay, Robot reached the pick-up location!");
    ros::Duration(5.0).sleep();

    //Going drop-off location
    goal.target_pose.pose.position.x = drop_off_location[0];
    goal.target_pose.pose.position.y = drop_off_location[1];
    goal.target_pose.pose.orientation.w = drop_off_location[2];

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending drop off goal.");
    ac.sendGoal(goal);
    // Wait an infinite time for the results
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, Robot reached the drop-off location!");
      ros::Duration(5.0).sleep();
    }
    else
    {
      ROS_INFO("Opps 404! Robot mysteriously failed to reach drop off zone :\(");
    }
  }
  else
  {
    ROS_INFO("Opps 404! Robot mysteriously failed to reach pick up zone :\(");
  }

  return 0;
}
