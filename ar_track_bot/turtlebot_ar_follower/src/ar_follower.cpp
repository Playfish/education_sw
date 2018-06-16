/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

#include <iostream>
#include <exception>
#include <algorithm>

#include <math.h>
namespace turtlebot_ar_follower
{

//* The turtlebot follower nodelet.
/**
 * The turtlebot follower nodelet. Subscribes to AlvarMarkers
 * from the ar_track_alvar, processes them, and publishes command vel
 * messages.
 */
class TurtlebotARFollower : public nodelet::Nodelet
{
public:
  /*!
   * @brief The constructor for the follower.
   * Constructor for the follower.
   */
  TurtlebotARFollower() : max_angular_speed_(2.0), min_angular_speed_(0.5),
                        max_x_(20.0), goal_x_(0.6),
                        x_threshold_(0.05), y_threshold_(0.05),
                        x_scale_(0.5), y_scale_(1.0),
                        max_linear_speed_(0.3), min_linear_speed_(0.1),
                        target_visible_(false)
  {

  }

  ~TurtlebotARFollower()
  {}

private:
  double max_x_; /**< The maximum distance a target can be from the robot for us to track. */
  double goal_x_; /**< The goal distance (in meters) to keep between the robot and the marker. */
  double x_threshold_; /**< How far away from the goal distance (in meters) before the robot reacts. */
  double y_threshold_; /**< How far away from being centered (y displacement) on the AR marker, before the robot reacts (units are meters) */
  double x_scale_; /**< How much do we weight the goal distance (x) when making a movement. */
  double y_scale_; /**< How much do we weight y-displacement when making a movement. */
  double max_linear_speed_; /**< The max linear speed in meters per second. */
  double min_linear_speed_; /**< The minimum linear speed in meters per second. */
  double max_angular_speed_; /**< The maximum rotation speed in radians per second. */
  double min_angular_speed_; /**< The minimum rotation speed in radians per second. */
  bool target_visible_; /**< Set flag to indicate when the AR marker is visible. */ 

  /*!
   * @brief OnInit method from node handle.
   * OnInit method from node handle. Sets up the parameters
   * and topics.
   */
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("max_angular_speed", max_angular_speed_);
    private_nh.getParam("min_angular_speed", min_angular_speed_);
    private_nh.getParam("max_x", max_x_);
    private_nh.getParam("goal_x", goal_x_);
    private_nh.getParam("x_threshold", x_threshold_);
    private_nh.getParam("y_threshold", y_threshold_);
    private_nh.getParam("x_scale", x_scale_);
    private_nh.getParam("y_scale", y_scale_);
    private_nh.getParam("max_linear_speed", max_linear_speed_);
    private_nh.getParam("min_linear_speed", min_linear_speed_);

    cmdpub_ = private_nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);

    sub_= nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 1, &TurtlebotARFollower::arposecb, this);

  }


  /*!
   * @brief Callback for ar track pose.
   * Subscribe to the ar_pose_marker topic 
   * to get the image width and height.
   * @param ar pose for ar track alvar.
   */
  void arposecb(const ar_track_alvar_msgs::AlvarMarkersConstPtr& msg)
  {
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    ar_track_alvar_msgs::AlvarMarker marker;
    if(msg->markers.size() !=0){
      marker = msg->markers[0];
      if(!target_visible_)        
        ROS_INFO_THROTTLE(1, "Ar Follower is tracking target!");
      target_visible_ = true;
    }else{
      // If target is loar, stop the robot by slowing it incrementally
      cmd->linear.x = 0;
      cmd->angular.z = 0;
      if(target_visible_)
        ROS_INFO_THROTTLE(1, "Ar Follower is lost target!");
      target_visible_ = false;
    }

    // Get the displacement of the marker relative to the base
    double target_offset_y = marker.pose.pose.position.y;

    // Get the distance of the marker from the base
    double target_offset_x = marker.pose.pose.position.x;

    // Rotate the robot only if the displacement of the target exceeds the threshold
    if(abs(target_offset_y) > y_threshold_){
      // Set the rotation speed proportional to the displacement of the target
      double speed = target_offset_y * y_scale_;
      cmd->angular.z = copysign(std::max(min_angular_speed_, std::min(max_angular_speed_, (double)abs(speed))), speed);

    }else{
      cmd->angular.z = 0.0;
    }

    // Now get the linear speed
    if(abs(target_offset_x) > x_threshold_){
      double speed = (target_offset_x - goal_x_) * x_scale_;
      if(speed < 0)
        speed *= 1.5;
      
      cmd->linear.x = copysign(std::min(max_linear_speed_, std::max(min_linear_speed_, (double)abs(speed))), speed);
    }else{
      
      cmd->angular.x = 0.0;
    }
    cmdpub_.publish(cmd);  
  }


  ros::Subscriber sub_;
  ros::Publisher cmdpub_;
};

PLUGINLIB_EXPORT_CLASS(turtlebot_ar_follower::TurtlebotARFollower, nodelet::Nodelet)

}
