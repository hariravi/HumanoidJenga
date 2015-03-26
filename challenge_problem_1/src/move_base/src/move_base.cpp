
#include "move_base.h"




int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
/*
// original code
  driver.driveForwardOdom(0.5);
  driver.turnOdom(true,3.14);
  driver.driveForwardOdom(0.5);
*/

  driver.driveForwardOdom(1.17);
  
}

