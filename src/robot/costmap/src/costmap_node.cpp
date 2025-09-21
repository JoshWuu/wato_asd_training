#include <chrono>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  // Subscribe to /lidar topic
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1)
  );
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),
    "LIDAR scan: angle_min=%.2f, angle_max=%.2f, increment=%.2f, ranges=%zu",
    msg->angle_min, msg->angle_max, msg->angle_increment, msg->ranges.size());
}

int main(int argc, char ** argv)  
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}