#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
  );
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::timerCallback, this)
  );
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = msg;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point& a,
                                    const geometry_msgs::msg::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if (!current_path_ || current_path_->poses.empty() || !current_odom_) {
    return std::nullopt;
  }

  const auto& robot_pos = current_odom_->pose.pose.position;

  for (const auto& pose : current_path_->poses) {
    if (computeDistance(pose.pose.position, robot_pos) >= LOOKAHEAD_DISTANCE) {
      return pose;
    }
  }

  // All waypoints are closer than lookahead — use the last one
  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(
  const geometry_msgs::msg::PoseStamped& target) {
  geometry_msgs::msg::Twist cmd;

  double yaw = extractYaw(current_odom_->pose.pose.orientation);
  double dx = target.pose.position.x - current_odom_->pose.pose.position.x;
  double dy = target.pose.position.y - current_odom_->pose.pose.position.y;

  // Transform lookahead point into robot's local frame
  double y_local = -std::sin(yaw) * dx + std::cos(yaw) * dy;

  double curvature = 2.0 * y_local / (LOOKAHEAD_DISTANCE * LOOKAHEAD_DISTANCE);

  cmd.linear.x = LINEAR_SPEED;
  cmd.angular.z = LINEAR_SPEED * curvature;

  return cmd;
}

void ControlNode::timerCallback() {
  if (!current_path_ || !current_odom_) return;
  if (current_path_->poses.empty()) return;

  // Stop if within tolerance of the final waypoint
  const auto& robot_pos = current_odom_->pose.pose.position;
  const auto& final_pos = current_path_->poses.back().pose.position;
  if (computeDistance(robot_pos, final_pos) < GOAL_TOLERANCE) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  auto lookahead = findLookaheadPoint();
  if (!lookahead) return;

  cmd_vel_pub_->publish(computeVelocity(*lookahead));
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
