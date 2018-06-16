/**
 * @file /speechbot/include/speechbot/speechbot.hpp
 *
 * @brief speech robot take out trash
 *
 * This is speech robot just take out trash based on navigation map.
 *
 * @author Carl
 *
 **/


/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <stdio.h>
#include <ros/callback_queue.h>
#include <fstream>
#include <sstream>
#include <std_msgs/Header.h>
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>  
#include <actionlib/client/simple_action_client.h> 
using namespace std;
namespace speechbot
{

class Speechbot : public nodelet::Nodelet
{
public:
  Speechbot() : goal_x_(0.0),goal_y_(0.0),
              goal_z_(0.0),ac("move_base", true){}

  ~Speechbot(){
  }
private:
  double goal_x_; /**< The position x of goal where speechbot speeching garbage. >*/
  double goal_y_; /**< The position y of goal where speechbot speeching garbage. >*/
  double goal_z_; /**< The position z of goal where speechbot speeching garbage. >*/
  double orien_z_; /**< The orientation z of goal where speechbot speeching garbage. >*/

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;
MoveBaseClient ac;
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    // Wait 60 seconds for the action server to become available
    ROS_INFO_STREAM("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(60));
    ROS_INFO_STREAM("Connected to move base server");

    private_nh.getParam("goal_posistion_x", goal_x_);
    private_nh.getParam("goal_posistion_y", goal_y_);
    private_nh.getParam("goal_posistion_z", goal_z_);
    private_nh.getParam("goal_orientation_z", orien_z_);
    ROS_INFO_STREAM("Init goal_position_x: "<<goal_x_<<" goal_position_y: "<<goal_y_<<" goal_position_z: "<<goal_z_<<" goal_orientation_z: "<<orien_z_<<" .");
    //goalpub_ = private_nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    speechSub_ = nh.subscribe<std_msgs::String>("/recognizer/output", 1, &Speechbot::recognizerOutputCb, this);
  }
  
  void sendGoal(const float goal_x, const float goal_y, const float orien_z){

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;
    goal.target_pose.pose.orientation.z = orien_z;
    goal.target_pose.pose.orientation.w = 1;
    ROS_INFO_STREAM("Sending goal:[ position.x: "<<goal_x<<", position.y: "<<goal_y<<", orientation.z: "<<orien_z<<" ].");
    ac.sendGoal(goal);
    // Wait for the action to return
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO_STREAM("Reach goal.");
    else
      ROS_INFO_STREAM("The base failed for some reason.");
  }


  int hexstring2char(const std::string& str, char* out){
    int n = 0;
    int temp;
    std::istringstream iss(str);
    iss >> std::hex;
    while (iss >> temp)
    {
        out[n++] = temp;
        iss.get();
    }
    return n;
  }

  void recognizerOutputCb(const std_msgs::String::ConstPtr& data){

    ROS_INFO_STREAM("Yes! I heard:[ " <<data->data<<" ].");

    // Origin recognizer output topic message.
    const char* getSendMsg=data->data.c_str();

    //Dictionary library for match message which come 
    //from recognizer output topic.
    const char* hello="hello";

    if(!strcmp(hello, getSendMsg)){
      sendGoal(1.0, 1.0, 0.7);
    }
    else{
      ROS_INFO_STREAM("Sorry, I cannot know where to go.");
    }
     
  }
    
  ros::Subscriber speechSub_;
  //ros::Publisher goalpub_;
  

};

PLUGINLIB_DECLARE_CLASS(speechbot, Speechbot, speechbot::Speechbot, nodelet::Nodelet);
}

