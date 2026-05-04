#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&MapMemoryNode::timerCallback, this)
  );

  initGlobalMap();
  map_pub_->publish(global_map_);
}

void MapMemoryNode::initGlobalMap() {
  global_map_.header.frame_id = "odom";
  global_map_.info.resolution = MAP_RESOLUTION;
  global_map_.info.width = MAP_WIDTH;
  global_map_.info.height = MAP_HEIGHT;
  global_map_.info.origin.position.x = -(MAP_WIDTH * MAP_RESOLUTION) / 2.0;
  global_map_.info.origin.position.y = -(MAP_HEIGHT * MAP_RESOLUTION) / 2.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(MAP_WIDTH * MAP_HEIGHT, -1);  // all unknown
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = msg;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  auto& q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  double dx = robot_x_ - last_update_x_;
  double dy = robot_y_ - last_update_y_;
  if (std::sqrt(dx * dx + dy * dy) >= UPDATE_DISTANCE) {
    should_update_map_ = true;
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
  }
}

void MapMemoryNode::timerCallback() {
  if (!should_update_map_ || !latest_costmap_) return;

  const auto& cm = *latest_costmap_;
  double cm_res = cm.info.resolution;
  int cm_w = static_cast<int>(cm.info.width);
  int cm_h = static_cast<int>(cm.info.height);
  double cm_ox = cm.info.origin.position.x;
  double cm_oy = cm.info.origin.position.y;

  double cos_yaw = std::cos(robot_yaw_);
  double sin_yaw = std::sin(robot_yaw_);
  double map_ox = global_map_.info.origin.position.x;
  double map_oy = global_map_.info.origin.position.y;

  // Transform each costmap cell into the global map frame
  for (int cy = 0; cy < cm_h; ++cy) {
    for (int cx = 0; cx < cm_w; ++cx) {
      int8_t val = cm.data[cy * cm_w + cx];

      // Local position of cell center in robot frame
      double lx = cm_ox + (cx + 0.5) * cm_res;
      double ly = cm_oy + (cy + 0.5) * cm_res;

      // Rotate and translate into odom frame
      double gx = robot_x_ + cos_yaw * lx - sin_yaw * ly;
      double gy = robot_y_ + sin_yaw * lx + cos_yaw * ly;

      // Convert to global grid index
      int mx = static_cast<int>((gx - map_ox) / MAP_RESOLUTION);
      int my = static_cast<int>((gy - map_oy) / MAP_RESOLUTION);

      if (mx >= 0 && mx < MAP_WIDTH && my >= 0 && my < MAP_HEIGHT) {
        // Overwrite with known value; retain previous if costmap cell is unknown (-1)
        if (val >= 0) {
          global_map_.data[my * MAP_WIDTH + mx] = val;
        }
      }
    }
  }

  global_map_.header.stamp = this->now();
  map_pub_->publish(global_map_);
  should_update_map_ = false;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
