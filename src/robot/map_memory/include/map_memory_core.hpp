#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Transforms local_costmap into the global frame using the robot's pose and merges it into the global map
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid & local_costmap, double robot_x, double robot_y, double robot_yaw);

    // Returns the current global map, stamped with the given time
    nav_msgs::msg::OccupancyGrid getGlobalMap(const rclcpp::Time & stamp) const;

  private:
    rclcpp::Logger logger_;

    // Global map parameters
    double resolution_;
    int width_;
    int height_;

    std::vector<std::vector<int8_t>> grid_;
};

}

#endif
