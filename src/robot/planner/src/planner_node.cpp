#include "planner_node.hpp"
#include <cmath>
#include <limits>

PlannerNode::PlannerNode() : Node("planner_node"),
    have_map(false), have_goal(false), have_pose(false)
{
  map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map = *msg;
  have_map = true;
  RCLCPP_INFO(this->get_logger(), "Received map: %d x %d", map.info.width, map.info.height);
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_point = *msg;
  have_goal = true;
  RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f)", msg->point.x, msg->point.y);
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose = *msg;
  have_pose = true;
}

void PlannerNode::timerCallback() {
  if (have_map && have_goal && have_pose) {
    if (!isGoalReached()) {
      RCLCPP_INFO(this->get_logger(), "Planning path...");
      planPath();
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
    }
  }
}

bool PlannerNode::isGoalReached() {
  double dx = goal_point.point.x - current_pose.pose.pose.position.x;
  double dy = goal_point.point.y - current_pose.pose.pose.position.y;
  double dist = std::sqrt(dx*dx + dy*dy);
  return dist < 0.3; // within 30 cm
}

double PlannerNode::heuristic(int x1, int y1, int x2, int y2) {
  // Euclidean
  return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void PlannerNode::planPath() {
  if (map.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "No map data available");
    return;
  }

  // TODO: implement full A* search here.
  // Right now just publish a trivial path: robot -> goal.

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header = path_msg.header;
  start_pose.pose = current_pose.pose.pose;

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = path_msg.header;
  goal_pose.pose.position = goal_point.point;
  goal_pose.pose.orientation.w = 1.0;

  path_msg.poses.push_back(start_pose);
  path_msg.poses.push_back(goal_pose);

  path_pub->publish(path_msg);
  RCLCPP_INFO(this->get_logger(), "Published simple path with start and goal.");
}

std::vector<std::pair<int,int>> PlannerNode::reconstructPath(
    std::unordered_map<int,int> &came_from, int start_id, int goal_id)
{
  std::vector<std::pair<int,int>> path;
  int current = goal_id;
  while (current != start_id) {
    int y = current / map.info.width;
    int x = current % map.info.width;
    path.push_back({x,y});
    current = came_from[current];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
