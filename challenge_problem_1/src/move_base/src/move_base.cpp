
#include "move_base.h"




int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle *nh = new ros::NodeHandle("");;

  ROS_INFO("About to initialize driver");
  RobotDriver driver(*nh);
  ROS_INFO("driver initiated");
  //driver.driveForwardOdom(1.5);
  ROS_INFO("ros information");
  ros::spin();
  ROS_INFO("SHOULD NEVER PRINT");
/*
// original code
  driver.driveForwardOdom(0.5);
  driver.turnOdom(true,3.14);
  driver.driveForwardOdom(0.5);
*/

  //driver.driveForwardOdom(1.17);  
  return 0;
}

