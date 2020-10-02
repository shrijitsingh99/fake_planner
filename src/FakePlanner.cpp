//
// Created by naivehobo on 7/7/19.
//

#include "fake_planner/FakePlanner.h"

#include <cmath>

FakePlanner::FakePlanner() : private_nh_("~"), tf_listener_(tf_buffer_)
{
  private_nh_.param<std::string>("goal_topic", goal_topic_, "move_base_simple/goal");
  private_nh_.param<std::string>("cmd_vel_topic", cmd_topic_, "cmd_vel");
  private_nh_.param<std::string>("goal_reached_topic", goal_reached_topic_, "goal_reached");
  private_nh_.param<std::string>("max_velocity_service", max_vel_service_name_, "set_max_velocity");
  private_nh_.param("planner_rate", planner_rate_, 10.0);
  private_nh_.param("max_linear_speed", max_linear_speed_, 0.5);
  private_nh_.param("max_angular_speed", max_angular_speed_, 0.75);
  private_nh_.param("position_tolerance", position_tolerance_, 0.1);
  private_nh_.param("orientation_tolerance", orientation_tolerance_, 0.1);
  private_nh_.param("heading_tolerance", heading_tolerance_, 0.8);
  private_nh_.param("enable_orientation_alignment", enable_orientation_alignment_, false);
  private_nh_.param("time_to_pos", time_to_pos_, 1.0);
  private_nh_.param("time_to_align", time_to_align_, 1.0);
  private_nh_.param("time_to_heading", time_to_heading_, 1.0);

  is_goal_set_ = false;

  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);
  goal_reached_pub_ = nh_.advertise<std_msgs::Bool>(goal_reached_topic_, 1);

  max_vel_server_ = private_nh_.advertiseService(max_vel_service_name_, &FakePlanner::setMaxVelocity, this);

  goal_sub_ = nh_.subscribe(goal_topic_, 1, &FakePlanner::goalCallback, this);
}

bool FakePlanner::setMaxVelocity(fake_planner::SetMaxVel::Request &req, fake_planner::SetMaxVel::Response &res)
{
  max_angular_speed_ = req.max_vel.data;
  max_linear_speed_ = req.max_vel.data;
}

void FakePlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr &goal)
{
  is_goal_set_ = true;
  goal_ = *goal;
  ROS_INFO("Goal was set to (%lf, %lf)",
           goal_.pose.position.x,
           goal_.pose.position.y);
}

void FakePlanner::run()
{

  geometry_msgs::PoseStamped goal_tf;
  geometry_msgs::TransformStamped transform;

  ros::Rate rate(planner_rate_);

  while (ros::ok())
  {

    geometry_msgs::Twist move_cmd;

    if (is_goal_set_)
    {

      try
      {

        transform = tf_buffer_.lookupTransform("base_link", goal_.header.frame_id, ros::Time(0), ros::Duration(1.0));

        tf2::doTransform(goal_, goal_tf, transform);

        double displacement = euclideanDistance(goal_tf.pose.position.x,
                                                goal_tf.pose.position.y);

        double heading = std::atan2(goal_tf.pose.position.y,
                                    goal_tf.pose.position.x);

        double roll, pitch, angular_displacement;
        tf2::Quaternion q(
            goal_tf.pose.orientation.x,
            goal_tf.pose.orientation.y,
            goal_tf.pose.orientation.z,
            goal_tf.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, angular_displacement);

        ROS_INFO("Displacement: %lf\tHeading: %lf", displacement, heading);

        if (displacement > position_tolerance_)
        {
          double linear_speed = displacement / time_to_pos_;

          double angular_speed = heading / time_to_heading_;

          if (std::abs(heading) > heading_tolerance_)
          {
            linear_speed = 0.0;
          }

          linear_speed = std::copysign(std::max(0.0, std::min(max_linear_speed_, std::abs(linear_speed))), linear_speed);
          angular_speed = std::copysign(std::max(0.0, std::min(max_angular_speed_, std::abs(angular_speed))), angular_speed);

          move_cmd.linear.x = linear_speed;
          move_cmd.angular.z = angular_speed;
        }
        else if (enable_orientation_alignment_ && std::abs(angular_displacement) > orientation_tolerance_)
        {
          ROS_INFO("Angular Displacement: %lf", angular_displacement);

          double angular_speed = angular_displacement / time_to_align_;

          angular_speed = std::copysign(std::max(0.0, std::min(0.4, std::abs(angular_speed))), angular_speed);

          move_cmd.angular.z = angular_speed;
        }
        else
        {
          ROS_INFO("Goal reached!");
          is_goal_set_ = false;
          std_msgs::Bool msg;
          msg.data = (u_char) true;
          goal_reached_pub_.publish(msg);
        }
      }
      catch (tf2::TransformException &e)
      {
        ROS_WARN("%s", e.what());
      }
    }

    cmd_pub_.publish(move_cmd);

    ros::spinOnce();

    rate.sleep();
  }
}
