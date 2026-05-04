#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  robot::CostmapCore costmap_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  static constexpr double RESOLUTION = 0.1;
  static constexpr int MAP_WIDTH = 200;
  static constexpr int MAP_HEIGHT = 200;
  static constexpr double INFLATION_RADIUS = 1.0;
};

#endif
