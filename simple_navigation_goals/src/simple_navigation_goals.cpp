#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "map_reader.c"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){


  ros::init(argc, argv, "simple_navigation_goals");
  
  ros::NodeHandle nh;

  ros::Rate loopRate(10);
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("/move_base", true);
  
  ROS_INFO("[SIMPLE_NAVIGATION_GOALS] Waiting for the move_base action server to come up");

  //wait for the action server to come up
  while(!ac.waitForServer()){}

  ROS_INFO("[SIMPLE_NAVIGATION_GOALS] move_base action available.");

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  int goals_x[1000];
  int goals_y[1000];

  int nav_points_num = get_rel_goals(goals_x, goals_y);
  int i;
  for (i = 0; i < nav_points_num && ros::ok(); i++) {
    loopRate.sleep();
    ros::spinOnce();
    goal.target_pose.pose.position.x = goals_x[i];
    goal.target_pose.pose.orientation.w = goals_y[i];

    ROS_INFO("Sending goal");
    ac.sendGoalAndWait(goal);
  }

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}