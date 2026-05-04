#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this)
  );
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  current_goal_ = msg;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", msg->point.x, msg->point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = msg;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_GOAL || !current_odom_ || !current_goal_) return;

  double dx = current_odom_->pose.pose.position.x - current_goal_->point.x;
  double dy = current_odom_->pose.pose.position.y - current_goal_->point.y;

  if (std::sqrt(dx * dx + dy * dy) < GOAL_THRESHOLD) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    return;
  }

  planPath();
}

void PlannerNode::planPath() {
  if (!current_map_ || !current_odom_ || !current_goal_) return;

  const auto& map = *current_map_;
  double res = map.info.resolution;
  int map_w = static_cast<int>(map.info.width);
  int map_h = static_cast<int>(map.info.height);
  double ox = map.info.origin.position.x;
  double oy = map.info.origin.position.y;

  double rx = current_odom_->pose.pose.position.x;
  double ry = current_odom_->pose.pose.position.y;
  double gx = current_goal_->point.x;
  double gy = current_goal_->point.y;

  CellIndex start{static_cast<int>((rx - ox) / res), static_cast<int>((ry - oy) / res)};
  CellIndex goal_cell{static_cast<int>((gx - ox) / res), static_cast<int>((gy - oy) / res)};

  auto in_bounds = [&](const CellIndex& c) {
    return c.x >= 0 && c.x < map_w && c.y >= 0 && c.y < map_h;
  };

  if (!in_bounds(start) || !in_bounds(goal_cell)) {
    RCLCPP_WARN(this->get_logger(), "Start or goal out of map bounds");
    return;
  }

  // Treat cells with cost >= threshold as obstacles; unknown (-1) is treated as free
  auto is_free = [&](const CellIndex& c) {
    int8_t val = map.data[c.y * map_w + c.x];
    return val < OCCUPIED_THRESHOLD;
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed;

  g_score[start] = 0.0;
  double h0 = std::hypot(goal_cell.x - start.x, goal_cell.y - start.y);
  open_list.push({start, h0});

  // 8-connected neighbors with diagonal cost
  const int ndx[] = {0, 0, 1, -1, 1, 1, -1, -1};
  const int ndy[] = {1, -1, 0, 0, 1, -1, 1, -1};
  const double step_costs[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

  bool found = false;

  while (!open_list.empty()) {
    AStarNode node = open_list.top();
    open_list.pop();

    if (closed.count(node.index)) continue;
    closed.insert(node.index);

    if (node.index == goal_cell) {
      found = true;
      break;
    }

    for (int i = 0; i < 8; ++i) {
      CellIndex nb{node.index.x + ndx[i], node.index.y + ndy[i]};
      if (!in_bounds(nb) || !is_free(nb) || closed.count(nb)) continue;

      double tentative_g = g_score[node.index] + step_costs[i];
      auto it = g_score.find(nb);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[nb] = tentative_g;
        came_from[nb] = node.index;
        double f = tentative_g + std::hypot(goal_cell.x - nb.x, goal_cell.y - nb.y);
        open_list.push({nb, f});
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(this->get_logger(), "A*: no path found to goal");
    return;
  }

  // Reconstruct path by walking back from goal to start
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = map.header.frame_id;

  std::vector<CellIndex> path_cells;
  CellIndex curr = goal_cell;
  while (curr != start) {
    path_cells.push_back(curr);
    curr = came_from[curr];
  }
  path_cells.push_back(start);
  std::reverse(path_cells.begin(), path_cells.end());

  for (const auto& cell : path_cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = ox + (cell.x + 0.5) * res;
    pose.pose.position.y = oy + (cell.y + 0.5) * res;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
