#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the node
  ros::init(argc, argv, "pick_objects");
  
  bool pickup_ok = false;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the pickup parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a pick-up position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -3.0;
  goal.target_pose.pose.position.y = 4.4;
 // goal.target_pose.pose.position.z = 0.0;

  // Robot only moves in 2D so it rotates around the z axis. Since z is "up", quaternions are (x, y, z, w) = (0, 0, sin(theta/2), cos(theta/2)).
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = std::sin(-0.5 * 1.4);
    goal.target_pose.pose.orientation.w = std::cos(-0.5 * 1.4);

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick-up goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Pick-up location reached");
	pickup_ok = true;
  }
  else
    ROS_INFO("Failed to reach pick-up location");
  
  if(pickup_ok){
    ros::Duration(5.0).sleep();
    ROS_INFO("Virtual object picked up");
	
    // set up the pickup parameters
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = -1.3;
    goal.target_pose.pose.position.y = 13.5;

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = std::sin(0.5 * 1.4);
    goal.target_pose.pose.orientation.w = std::cos(0.5 * 1.4);

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending drop-off goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Drop-off location reached");    
    else
      ROS_INFO("Failed to reach drop-off location");
  }

  return 0;
}
