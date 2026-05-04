#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();
  void initGlobalMap();

  robot::MapMemoryCore map_memory_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  nav_msgs::msg::OccupancyGrid global_map_;

  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  double last_update_x_{0.0};
  double last_update_y_{0.0};
  bool should_update_map_{false};

  static constexpr double MAP_RESOLUTION = 0.1;
  static constexpr int MAP_WIDTH = 500;   // 50 m
  static constexpr int MAP_HEIGHT = 500;  // 50 m
  static constexpr double UPDATE_DISTANCE = 1.5;
};

#endif
