#include <moveit/move_group_interface/move_group.h>
#include <iostream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "move_arm_example");
  ros::NodeHandle node_handle;


  moveit::planning_interface::MoveGroup l_arm_move_group("left_arm");


  // as an example, we can set a random target
  l_arm_move_group.setRandomTarget();
  l_arm_move_group.move();

  //but here is how to set a specific pose goal
  // right arm
// quaternion -> orientation
  geometry_msgs::Pose goal_end_effector_pose;
// Orientation: 4 numbers
  goal_end_effector_pose.orientation.w = 1.0;
// 3D space: 3 numbers (meters)
  goal_end_effector_pose.position.x = 0.28; // forward
  goal_end_effector_pose.position.y = 0.7; // left
  goal_end_effector_pose.position.z = 1.0; // up
  l_arm_move_group.setPoseTarget(goal_end_effector_pose);


  // plan the motion and then move the group to the sampled target 
  l_arm_move_group.move();


  return 0;
}
