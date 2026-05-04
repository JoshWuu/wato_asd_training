#include <cmath>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1)
  );
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Grid initialized to 0 (free)
  std::vector<std::vector<int>> grid(MAP_HEIGHT, std::vector<int>(MAP_WIDTH, 0));

  // Origin places the robot at the center of the costmap
  double origin_x = -MAP_WIDTH * RESOLUTION / 2.0;
  double origin_y = -MAP_HEIGHT * RESOLUTION / 2.0;

  // Mark occupied cells from laser scan
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
      continue;
    }
    double angle = scan->angle_min + i * scan->angle_increment;
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    int gx = static_cast<int>((x - origin_x) / RESOLUTION);
    int gy = static_cast<int>((y - origin_y) / RESOLUTION);

    if (gx >= 0 && gx < MAP_WIDTH && gy >= 0 && gy < MAP_HEIGHT) {
      grid[gy][gx] = 100;
    }
  }

  // Inflate obstacles
  int inflation_cells = static_cast<int>(INFLATION_RADIUS / RESOLUTION);
  std::vector<std::vector<int>> inflated = grid;

  for (int gy = 0; gy < MAP_HEIGHT; ++gy) {
    for (int gx = 0; gx < MAP_WIDTH; ++gx) {
      if (grid[gy][gx] != 100) continue;
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          int ny = gy + dy;
          int nx = gx + dx;
          if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) continue;
          double dist = std::sqrt(dx * dx + dy * dy) * RESOLUTION;
          if (dist > INFLATION_RADIUS) continue;
          int cost = static_cast<int>(100.0 * (1.0 - dist / INFLATION_RADIUS));
          inflated[ny][nx] = std::max(inflated[ny][nx], cost);
        }
      }
    }
  }

  // Build and publish OccupancyGrid
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "robot";
  msg.info.resolution = RESOLUTION;
  msg.info.width = MAP_WIDTH;
  msg.info.height = MAP_HEIGHT;
  msg.info.origin.position.x = origin_x;
  msg.info.origin.position.y = origin_y;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(MAP_WIDTH * MAP_HEIGHT);

  for (int gy = 0; gy < MAP_HEIGHT; ++gy) {
    for (int gx = 0; gx < MAP_WIDTH; ++gx) {
      msg.data[gy * MAP_WIDTH + gx] = static_cast<int8_t>(inflated[gy][gx]);
    }
  }

  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
