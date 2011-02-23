#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <move_arm_msgs/utils.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_right_arm",true);
  move_arm.waitForServer();
  ROS_INFO("Connected to server");
  move_arm_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 3;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  motion_planning_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = "base_link";
  desired_pose.link_name = "grippoint_right";
  desired_pose.pose.position.x = 0.4;
  desired_pose.pose.position.y = -0.15;
  desired_pose.pose.position.z = 0.5;

/*
 0.32;
  -0.33;
   0.5;
*/

   	geometry_msgs::Quaternion quat_msg = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

  desired_pose.pose.orientation.x = quat_msg.x;
  desired_pose.pose.orientation.y = quat_msg.y;
  desired_pose.pose.orientation.z = quat_msg.z;
  desired_pose.pose.orientation.w = quat_msg.w;

  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;

  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;
  
  move_arm_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);
  for (int i = 0; i < 10 ; i++){
  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(30.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success){
        ROS_INFO("Action A finished: %s",state.toString().c_str());
        break;
	  }
      else
        ROS_INFO("Action A failed: %s",state.toString().c_str());
    }
  }
}
/*

  ////////////////////// second goal ///////////////////////////////////
  usleep(3000000);
  
  desired_pose.header.frame_id = "base_link";
  desired_pose.link_name = "grippoint_right";
  desired_pose.pose.position.x = 0.2;
  desired_pose.pose.position.y = -0.2;
  desired_pose.pose.position.z = 0.3;  
  
  quat_msg = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

  desired_pose.pose.orientation.x = quat_msg.x;
  desired_pose.pose.orientation.y = quat_msg.y;
  desired_pose.pose.orientation.z = quat_msg.z;
  desired_pose.pose.orientation.w = quat_msg.w;
  
    
  for (int i = 0; i < 10 ; i++){  
  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 3;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  move_arm_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(30.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success){
        ROS_INFO("Action B finished: %s",state.toString().c_str());
        break;
	  }
      else
        ROS_INFO("Action B failed: %s",state.toString().c_str());
    }
  }
  
  }

  */
  /*
  ////////////////////// third goal ///////////////////////////////////
  usleep(2000000);
  
  desired_pose.header.frame_id = "base_link";
  desired_pose.link_name = "grippoint_right";
  desired_pose.pose.position.x = 0.33;
  desired_pose.pose.position.y = -0.33;
  desired_pose.pose.position.z = 0.5;  
  
  quat_msg = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -0.5);

  desired_pose.pose.orientation.x = quat_msg.x;
  desired_pose.pose.orientation.y = quat_msg.y;
  desired_pose.pose.orientation.z = quat_msg.z;
  desired_pose.pose.orientation.w = quat_msg.w;
  
    move_arm_msgs::MoveArmGoal goalC;
    
  for (int i = 0; i < 10 ; i++){  
  goalC.motion_plan_request.group_name = "right_arm";
  goalC.motion_plan_request.num_planning_attempts = 1;
  goalC.motion_plan_request.planner_id = std::string("");
  goalC.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalC.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  move_arm_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalC);

  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalC);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal C");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success){
        ROS_INFO("Action finished: %s",state.toString().c_str());
        break;
	  }
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  
  } 
  */
  ros::shutdown();
}
