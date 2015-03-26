#include <ros/ros.h>
#include "move_base/MoveBase.h"


int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "pickup_object");

  ros::NodeHandle nh;


// set up ros::ServiceClient
  ros::ServiceClient client = nh.serviceClient<move_base::MoveBase>("/move_base");
  
  move_base::MoveBase srv;
  srv.request.distance = 1.17;
  
if (client.call(srv))
{
ROS_INFO("Success: %d", (int) srv.response.success);
}
else{
ROS_ERROR("Failed to move forward");
return 1;
}


/*
  //sleep for a moment while everything starts up
  ros::Duration(1.0).sleep();

  //your code here
  RobotDriver driver(nh);
	fprintf(stderr, "RobotDriver initialized\n");
  driver.driveForwardOdom(0.5);
  driver.turnOdom(true,3.14);
  driver.driveForwardOdom(0.5);



  ROS_INFO("The hair of Renato");
*/
  return 0;
}
