#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstdint>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Runs A* over the occupancy grid from (start_x, start_y) to (goal_x, goal_y), both in the
    // map's world frame. Returns a Path stamped/framed to match map; poses are empty if no path exists.
    nav_msgs::msg::Path computePath(
      const nav_msgs::msg::OccupancyGrid & map,
      double start_x, double start_y,
      double goal_x, double goal_y,
      const rclcpp::Time & stamp) const;

    // Cells at or above this cost are treated as obstacles; unknown (-1) cells are also non-traversable
    static constexpr int8_t kLethalCost = 50;

  private:
    rclcpp::Logger logger_;
};

}

#endif
