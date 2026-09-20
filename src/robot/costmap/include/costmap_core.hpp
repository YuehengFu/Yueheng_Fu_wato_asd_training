#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Builds a local occupancy grid from a laser scan
    nav_msgs::msg::OccupancyGrid processScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

  private:
    void initializeCostmap();
    void convertToGrid(double range, double angle, int & x_grid, int & y_grid) const;
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();

    rclcpp::Logger logger_;

    // Costmap parameters
    double resolution_;
    int width_;
    int height_;
    double inflation_radius_;
    int8_t max_cost_;

    std::vector<std::vector<int8_t>> grid_;
};

}

#endif
