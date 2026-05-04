#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
  geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target);
  double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
  double extractYaw(const geometry_msgs::msg::Quaternion& q);

  robot::ControlCore control_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path::SharedPtr current_path_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;

  static constexpr double LOOKAHEAD_DISTANCE = 1.0;
  static constexpr double LINEAR_SPEED = 0.5;
  static constexpr double GOAL_TOLERANCE = 0.3;
};

#endif
