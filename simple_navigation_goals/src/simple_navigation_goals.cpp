#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void commandCallback(const std_msgs::String::ConstPtr& msg)
{
  //tell user what we heard
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

  move_base_msgs::MoveBaseGoal goal;

  if(msg->data.c_str() == "bathroom")
    {
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.pose.position.x = -3.279;
      goal.target_pose.pose.position.y = 2.926;
      goal.target_pose.pose.orientation.w = 1;
    }
  else if(msg->data.c_str() == "341")
    {
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.pose.position.x = -3.298;
      goal.target_pose.pose.position.y = -1.016;
      goal.target_pose.pose.orientation.w = 1;
    }
  else if(msg->data.c_str() == "342")
    {
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.pose.position.x = 1.946;
      goal.target_pose.pose.position.y = -0.276;
      goal.target_pose.pose.orientation.w = 1;
    }
  else
    {
      ROS_INFO("Not a valid choice");
    }

  //send the goal
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();
   
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("We're here!");
  else
    ROS_INFO("We couldn't get there.");
}

int main(int argc, char** argv)
{
  //ROS initialization
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n;

  //set up the node to subscribe to the "husky_nav" topic
  ros::Subscriber sub = n.subscribe("husky_nav", 5, commandCallback);

  ros::spin();

  return 0;
}
