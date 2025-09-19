#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  // Timed publisher
  void publishMessage();

  // LaserScan callback
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  // Costmap helper
  robot::CostmapCore costmap;

  // ROS2 constructs
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
  rclcpp::TimerBase::SharedPtr timer;
};
#endif