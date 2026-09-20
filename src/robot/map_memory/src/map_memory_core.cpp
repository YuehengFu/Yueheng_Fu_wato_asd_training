#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger),
    resolution_(0.1),
    width_(600),
    height_(600) {
  grid_.assign(height_, std::vector<int8_t>(width_, -1));
}

void MapMemoryCore::integrateCostmap(
  const nav_msgs::msg::OccupancyGrid & local_costmap, double robot_x, double robot_y, double robot_yaw) {
  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);

  double global_origin_x = -(width_ / 2.0) * resolution_;
  double global_origin_y = -(height_ / 2.0) * resolution_;

  for (unsigned int row = 0; row < local_costmap.info.height; ++row) {
    for (unsigned int col = 0; col < local_costmap.info.width; ++col) {
      int8_t value = local_costmap.data[row * local_costmap.info.width + col];
      if (value < 0) {
        continue;  // unknown cell, nothing to merge
      }

      // Cell center in the local costmap's own frame
      double local_x = local_costmap.info.origin.position.x + (col + 0.5) * local_costmap.info.resolution;
      double local_y = local_costmap.info.origin.position.y + (row + 0.5) * local_costmap.info.resolution;

      // Rotate and translate into the global (world) frame using the robot's pose
      double world_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
      double world_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

      int global_col = static_cast<int>(std::floor((world_x - global_origin_x) / resolution_));
      int global_row = static_cast<int>(std::floor((world_y - global_origin_y) / resolution_));

      if (global_col < 0 || global_col >= width_ || global_row < 0 || global_row >= height_) {
        continue;
      }

      // Prioritize new data over old data
      grid_[global_row][global_col] = value;
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap(const rclcpp::Time & stamp) const {
  nav_msgs::msg::OccupancyGrid global_map;
  global_map.header.stamp = stamp;
  global_map.header.frame_id = "sim_world";

  global_map.info.resolution = static_cast<float>(resolution_);
  global_map.info.width = width_;
  global_map.info.height = height_;
  global_map.info.origin.position.x = -(width_ / 2.0) * resolution_;
  global_map.info.origin.position.y = -(height_ / 2.0) * resolution_;
  global_map.info.origin.position.z = 0.0;
  global_map.info.origin.orientation.w = 1.0;

  global_map.data.resize(static_cast<size_t>(width_) * static_cast<size_t>(height_));
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      global_map.data[static_cast<size_t>(y) * width_ + x] = grid_[y][x];
    }
  }

  return global_map;
}

}
