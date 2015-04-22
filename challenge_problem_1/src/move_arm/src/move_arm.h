
#ifndef MOVE_ARM_H
#define MOVE_BASE_H

#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include "move_arm/MoveRightArm.h"
#include "move_arm/MoveLeftArm.h"

class RobotArm
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! our arms
  //moveit::planning_interface::MoveGroup l_arm_move_group;
  //moveit::planning_interface::MoveGroup r_arm_move_group;
  //! our services
  ros::ServiceServer service1_;
  ros::ServiceServer service2_;

public:
  //! ROS node init
  RobotArm(ros::NodeHandle &nh)
  {
    nh_ = nh;
    // init our arms
    //l_arm_move_group = moveit::planning_interface::MoveGroup("left_arm");
    //r_arm_move_group = moveit::planning_interface::MoveGroup("right_arm");
    // advertise service
    service1_ = nh_.advertiseService("/move_right_arm", &RobotArm::rightArm_cb, this);
    service2_ = nh_.advertiseService("/move_left_arm", &RobotArm::leftArm_cb, this);

  }

bool rightArm_cb(move_arm::MoveRightArm::Request &req, move_arm::MoveRightArm::Response &res)
{
  geometry_msgs::Pose goal_end_effector_pose;
  // set end effector pose
  goal_end_effector_pose.orientation.w = req.ow;
  goal_end_effector_pose.orientation.x = req.ox;
  goal_end_effector_pose.orientation.y = req.oy;
  goal_end_effector_pose.orientation.z = req.oz;
  goal_end_effector_pose.position.x = req.x;
  goal_end_effector_pose.position.y = req.y;
  goal_end_effector_pose.position.z = req.z;

  // set end effector pose
  moveit::planning_interface::MoveGroup r_arm_move_group("right_arm");
  r_arm_move_group.setPoseReferenceFrame("base_link");
  r_arm_move_group.setPoseTarget(goal_end_effector_pose);
  //r_arm_move_group.setPositionTarget(req.x, req.y, req.z);
  //r_arm_move_group.setRPYTarget(req.ox, req.oy, req.oz);
  r_arm_move_group.asyncMove();
  
  
  // TODO: don't just return success
  res.success = true;
  return res.success;
}

bool leftArm_cb(move_arm::MoveLeftArm::Request &req, move_arm::MoveLeftArm::Response &res)
{
  geometry_msgs::Pose goal_end_effector_pose;
  // set end effector pose
  goal_end_effector_pose.orientation.w = req.ow;
  goal_end_effector_pose.orientation.x = req.ox;
  goal_end_effector_pose.orientation.y = req.oy;
  goal_end_effector_pose.orientation.z = req.oz;
  goal_end_effector_pose.position.x = req.x;
  goal_end_effector_pose.position.y = req.y;
  goal_end_effector_pose.position.z = req.z;

  // set end effector pose
  moveit::planning_interface::MoveGroup l_arm_move_group("left_arm");
  l_arm_move_group.setPoseReferenceFrame("base_link");
  l_arm_move_group.setPoseTarget(goal_end_effector_pose);
  //l_arm_move_group.setPositionTarget(req.x, req.y, req.z);
  //l_arm_move_group.setRPYTarget(req.ox, req.oy, req.oz);
  l_arm_move_group.asyncMove();
  
  
  // TODO: don't just return success
  res.success = true;
  return res.success;
}

};

#endif

