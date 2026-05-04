#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "planner_core.hpp"

struct CellIndex {
  int x, y;
  bool operator==(const CellIndex& o) const { return x == o.x && y == o.y; }
  bool operator!=(const CellIndex& o) const { return !(*this == o); }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex& c) const {
    return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 16);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;
};

struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) const {
    return a.f_score > b.f_score;  // min-heap
  }
};

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();
  void planPath();

  robot::PlannerCore planner_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  geometry_msgs::msg::PointStamped::SharedPtr current_goal_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;

  State state_{State::WAITING_FOR_GOAL};

  static constexpr double GOAL_THRESHOLD = 0.5;
  static constexpr int OCCUPIED_THRESHOLD = 50;
};

#endif
