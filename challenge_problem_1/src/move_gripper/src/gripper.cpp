#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include "move_gripper/CloseGripper.h"
#include "move_gripper/CloseRightGripper.h"

class Gripper{
private:
  ros::NodeHandle nh_;
  ros::Publisher l_gripper_;
  ros::Publisher r_gripper_;
  tf::TransformListener listener_;
  ros::ServiceServer service;
ros::ServiceServer serviceRight;


public:
  //Action client initialization
  Gripper(ros::NodeHandle &nh){
    nh_ = nh;
    l_gripper_ = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/l_gripper_controller/command", 1);
    r_gripper_ = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/r_gripper_controller/command", 1);


    // advertise service
    service = nh_.advertiseService("/move_gripper", &Gripper::closeGripper_callback, this);
serviceRight = nh_.advertiseService("/move_right_gripper", &Gripper::closeRightGripper_callback, this);

    ROS_INFO("We will start in one minute. Hari, you're not welcome to class.");
  }

  ~Gripper(){
  }

bool closeRightGripper_callback(move_gripper::CloseRightGripper::Request &req,
         move_gripper::CloseRightGripper::Response &res)
{
    if (req.close == true) {
        res.success = closeRight();
    }
    else {
        res.success = openRight();
    }
    return res.success;
}

bool closeGripper_callback(move_gripper::CloseGripper::Request &req,
         move_gripper::CloseGripper::Response &res)
{
    if (req.close == true) {
        res.success = close();
    }
    else {
        res.success = open();
    }
    return res.success;
}

//Open the right gripper
  bool openRight(){

    ROS_INFO("Opening Right Gripper");
    ros::Duration(0.5).sleep();

    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "r_gripper_r_finger_tip_frame", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the gripper to the base frame
    listener_.lookupTransform("base_footprint", "r_gripper_r_finger_tip_frame", 
                              ros::Time(0), start_transform);

    bool done = false;
    pr2_controllers_msgs::Pr2GripperCommand gripper_cmd;
    gripper_cmd.position = 0.09;
    gripper_cmd.max_effort = -1.0;

    ros::Rate rate(10.0);


    while (!done && nh_.ok())
    {
      r_gripper_.publish(gripper_cmd);

      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "r_gripper_r_finger_tip_frame", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how if the gripper is open
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();
      //ROS_INFO("%f",dist_moved);
      if(dist_moved > 0.04) done = true;
    }
    return true;
  }

//Close the gripper
  bool closeRight(){
    
    ROS_INFO("Closing Right Gripper, hair of Renato");
    ros::Duration(0.5).sleep();

    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "r_gripper_r_finger_tip_frame", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the gripper to the base frame
    listener_.lookupTransform("base_footprint", "r_gripper_r_finger_tip_frame", 
                              ros::Time(0), start_transform);

    bool done = false;
    pr2_controllers_msgs::Pr2GripperCommand gripper_cmd;
    gripper_cmd.position = 0.0;
    gripper_cmd.max_effort = 50.0;

    ros::Rate rate(10.0);

    double dist_moved_before;
    double dist_moved;
    while (!done && nh_.ok())
    {
      r_gripper_.publish(gripper_cmd);

      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "r_gripper_r_finger_tip_frame", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
  
      //see how if the gripper is open or if it hit some object
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;

      dist_moved_before = dist_moved;
      dist_moved = relative_transform.getOrigin().length();

      //ROS_INFO("%f",dist_moved);
      if(dist_moved > 0.03 || dist_moved < dist_moved_before) done = true;
    
    }
    return true;
  }



  //Open the gripper
  bool open(){

    ROS_INFO("Opening Gripper");
    ros::Duration(0.5).sleep();

    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "l_gripper_l_finger_tip_frame", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the gripper to the base frame
    listener_.lookupTransform("base_footprint", "l_gripper_l_finger_tip_frame", 
                              ros::Time(0), start_transform);

    bool done = false;
    pr2_controllers_msgs::Pr2GripperCommand gripper_cmd;
    gripper_cmd.position = 0.09;
    gripper_cmd.max_effort = -1.0;

    ros::Rate rate(10.0);


    while (!done && nh_.ok())
    {
      l_gripper_.publish(gripper_cmd);

      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "l_gripper_l_finger_tip_frame", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how if the gripper is open
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();
      //ROS_INFO("%f",dist_moved);
      if(dist_moved > 0.04) done = true;
    }
    return true;
  }

  //Close the gripper
  bool close(){
    
    ROS_INFO("Closing Gripper, hair of Renato");
    ros::Duration(0.5).sleep();

    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "l_gripper_l_finger_tip_frame", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the gripper to the base frame
    listener_.lookupTransform("base_footprint", "l_gripper_l_finger_tip_frame", 
                              ros::Time(0), start_transform);

    bool done = false;
    pr2_controllers_msgs::Pr2GripperCommand gripper_cmd;
    gripper_cmd.position = 0.0;
    gripper_cmd.max_effort = 50.0;

    ros::Rate rate(10.0);

    double dist_moved_before;
    double dist_moved;
    while (!done && nh_.ok())
    {
      l_gripper_.publish(gripper_cmd);

      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "l_gripper_l_finger_tip_frame", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
  
      //see how if the gripper is open or if it hit some object
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;

      dist_moved_before = dist_moved;
      dist_moved = relative_transform.getOrigin().length();

      //ROS_INFO("%f",dist_moved);
      if(dist_moved > 0.03 || dist_moved < dist_moved_before) done = true;
    
    }
    return true;
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_gripper");
  ros::NodeHandle nh;

  Gripper gripper(nh);
  ros::spin();

  return 0;
}


