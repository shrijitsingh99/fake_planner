/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Sarthak Mittal
 *
 */

#pragma once

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <fake_planner/SetMaxVel.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>

class FakePlanner {

 public:
  FakePlanner();

  void run();

  void goalCallback(const geometry_msgs::PoseStampedConstPtr &goal);
  bool setMaxVelocity(fake_planner::SetMaxVel::Request  &req,
                      fake_planner::SetMaxVel::Response &res);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher cmd_pub_;
  ros::Publisher goal_reached_pub_;

  ros::Subscriber goal_sub_;

  ros::ServiceServer max_vel_server_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double planner_rate_;
  double max_angular_speed_;
  double max_linear_speed_;
  double position_tolerance_;
  double orientation_tolerance_;
  double heading_tolerance_;
  double time_to_pos_;
  double time_to_heading_;
  double time_to_align_;
  bool is_goal_set_;
  bool enable_orientation_alignment_;

  std::string goal_topic_;
  std::string cmd_topic_;
  std::string goal_reached_topic_;
  std::string max_vel_service_name_;

  geometry_msgs::PoseStamped goal_;

  double euclideanDistance(double x, double y) {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
  }

};
