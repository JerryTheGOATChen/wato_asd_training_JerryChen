#ifndef MAP_MEMORY_NODE_HPP
#define MAP_MEMORY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  // Callbacks
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void updateMap();

  // ROS constructs
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
  rclcpp::TimerBase::SharedPtr timer;

  // Stored maps
  nav_msgs::msg::OccupancyGrid latest_costmap;
  nav_msgs::msg::OccupancyGrid global_map;

  // Robot position tracking
  float last_x, last_y;
  float distance_threshold;
  bool should_update_map;

  // Core object
  robot::MapMemoryCore map_memory;
};

#endif
