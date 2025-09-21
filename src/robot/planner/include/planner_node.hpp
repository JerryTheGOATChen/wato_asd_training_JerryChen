#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <vector>
#include <queue>
#include <unordered_map>

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  // Callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // Helpers
  void planPath();
  bool isGoalReached();
  double heuristic(int x1, int y1, int x2, int y2);
  std::vector<std::pair<int,int>> reconstructPath(
      std::unordered_map<int,int> &came_from, int start_id, int goal_id);

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

  // Timer
  rclcpp::TimerBase::SharedPtr timer;

  // Stored data
  nav_msgs::msg::OccupancyGrid map;
  geometry_msgs::msg::PointStamped goal_point;
  nav_msgs::msg::Odometry current_pose;

  bool have_map;
  bool have_goal;
  bool have_pose;
};

#endif