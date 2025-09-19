#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode():Node("map_memory"), last_x(0.0), last_y(0.0), distance_threshold(1.5), should_update_map(false), map_memory(this->get_logger())  // initialize core object
{
  // Subscribe to costmap
  costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );

  // Subscribe to odom
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  // Publisher
  map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Timer
  timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap = *msg;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  float x_current = msg->pose.pose.position.x;
  float y_current = msg->pose.pose.position.y;

  float dx = x_current - last_x;
  float dy = y_current - last_y;
  float distance = std::sqrt(dx*dx + dy*dy);

  RCLCPP_INFO(this->get_logger(), "dx=%.2f, dy=%.2f, distance=%.2f", dx, dy, distance);

  if (distance >= distance_threshold) {
    should_update_map = true;
    last_x = x_current;
    last_y = y_current;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map && !latest_costmap.data.empty()) {
    global_map = latest_costmap; // TODO: merge instead of overwrite
    map_pub->publish(global_map_);
    should_update_map = false;
    RCLCPP_INFO(this->get_logger(), "Published updated global map.");
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
