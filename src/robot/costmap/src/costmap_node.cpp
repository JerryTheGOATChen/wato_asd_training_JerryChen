#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode(): Node("costmap"), costmap(robot::CostmapCore(this->get_logger()))
{
  // Timer publisher
  //string_pub = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);        Testing
  //timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  // LaserScan subscription
  lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  // Costmap publisher
  costmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

/*void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub->publish(message);
}
*/

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  float resolution = 0.1;
  int width = 300;      //Maybe change later because we only need local map not whole global
  int height = 300;

  std::vector<std::vector<int>> costmap(height, std::vector<int>(width, 0));

  // LaserScan -> grid
  for (int i = 0; i < scan->ranges.size(); ++i) {
    float angle = scan->angle_min + i * scan->angle_increment;
    float range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) {
      float x = range * cos(angle);
      float y = range * sin(angle);

      int grid_x = static_cast<int>((x / resolution) + width / 2);
      int grid_y = static_cast<int>((y / resolution) + height / 2);

      if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
        costmap[grid_y][grid_x] = 100;
      }
    }
  }

  // Inflate obstacles
  float inflation_radius = 0.3;
  int inflation_cells = static_cast<int>(inflation_radius / resolution);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (costmap[y][x] == 100) {
        for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
          for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
              float distance = std::sqrt(dx*dx + dy*dy) * resolution;
              if (distance <= inflation_radius) {
                int cost = 100 * (1 - distance / inflation_radius);
                costmap[ny][nx] = std::max(costmap[ny][nx], cost);
              }
            }
          }
        }
      }
    }
  }

  // Publish OccupancyGrid
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.stamp = this->get_clock()->now();
  grid_msg.header.frame_id = "map";
  grid_msg.info.resolution = resolution;
  grid_msg.info.width = width;
  grid_msg.info.height = height;
  grid_msg.info.origin.position.x = -(width * resolution) / 2.0;
  grid_msg.info.origin.position.y = -(height * resolution) / 2.0;
  grid_msg.data.resize(width * height);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      grid_msg.data[y * width + x] = costmap[y][x];
    }
  }

  costmap_pub->publish(grid_msg);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}