#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // Callback for costmap updates
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    // Callback for odometry updates
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // Timer-based map update
    void updateMap();

    robot::MapMemoryCore map_memory_;

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot pose at the last map update, and the distance threshold to trigger the next one
    double last_x_;
    double last_y_;
    double last_yaw_;
    const double distance_threshold_;

    // Latest costmap and update flags
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool costmap_updated_;
    bool should_update_map_;
};

#endif
