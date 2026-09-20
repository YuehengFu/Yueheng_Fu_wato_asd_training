#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger),
    resolution_(0.1),
    width_(300),
    height_(300),
    inflation_radius_(1.0),
    max_cost_(100) {}

void CostmapCore::initializeCostmap() {
  grid_.assign(height_, std::vector<int8_t>(width_, 0));
}

void CostmapCore::convertToGrid(double range, double angle, int & x_grid, int & y_grid) const {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  // Origin is the center of the grid, so the sensor sits at (width_/2, height_/2)
  x_grid = static_cast<int>(std::round(x / resolution_)) + width_ / 2;
  y_grid = static_cast<int>(std::round(y / resolution_)) + height_ / 2;
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    grid_[y_grid][x_grid] = max_cost_;
  }
}

void CostmapCore::inflateObstacles() {
  int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  // Snapshot the obstacles first so inflation doesn't cascade off cells we just inflated
  std::vector<std::pair<int, int>> obstacles;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_[y][x] == max_cost_) {
        obstacles.emplace_back(x, y);
      }
    }
  }

  for (const auto & obstacle : obstacles) {
    int ox = obstacle.first;
    int oy = obstacle.second;

    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        int nx = ox + dx;
        int ny = oy + dy;
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
          continue;
        }

        double distance = std::hypot(dx, dy) * resolution_;
        if (distance > inflation_radius_) {
          continue;
        }

        int8_t cost = static_cast<int8_t>(max_cost_ * (1.0 - distance / inflation_radius_));
        if (cost > grid_[ny][nx]) {
          grid_[ny][nx] = cost;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::processScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
  initializeCostmap();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (range < scan->range_min || range > scan->range_max) {
      continue;
    }

    double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    int x_grid, y_grid;
    convertToGrid(range, angle, x_grid, y_grid);
    markObstacle(x_grid, y_grid);
  }

  inflateObstacles();

  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header = scan->header;
  occupancy_grid.info.resolution = static_cast<float>(resolution_);
  occupancy_grid.info.width = width_;
  occupancy_grid.info.height = height_;
  occupancy_grid.info.origin.position.x = -(width_ / 2.0) * resolution_;
  occupancy_grid.info.origin.position.y = -(height_ / 2.0) * resolution_;
  occupancy_grid.info.origin.position.z = 0.0;
  occupancy_grid.info.origin.orientation.w = 1.0;

  occupancy_grid.data.resize(static_cast<size_t>(width_) * static_cast<size_t>(height_));
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      occupancy_grid.data[static_cast<size_t>(y) * width_ + x] = grid_[y][x];
    }
  }

  return occupancy_grid;
}

}
