#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "planner_core.hpp"

namespace robot
{

namespace
{

bool isFree(const nav_msgs::msg::OccupancyGrid & map, const CellIndex & cell)
{
  if (cell.x < 0 || cell.x >= static_cast<int>(map.info.width) ||
      cell.y < 0 || cell.y >= static_cast<int>(map.info.height)) {
    return false;
  }
  int8_t value = map.data[static_cast<size_t>(cell.y) * map.info.width + static_cast<size_t>(cell.x)];
  return value >= 0 && value < PlannerCore::kLethalCost;
}

CellIndex worldToGrid(const nav_msgs::msg::OccupancyGrid & map, double x, double y)
{
  int gx = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
  int gy = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
  return CellIndex(gx, gy);
}

geometry_msgs::msg::Point gridToWorld(const nav_msgs::msg::OccupancyGrid & map, const CellIndex & cell)
{
  geometry_msgs::msg::Point p;
  p.x = map.info.origin.position.x + (cell.x + 0.5) * map.info.resolution;
  p.y = map.info.origin.position.y + (cell.y + 0.5) * map.info.resolution;
  p.z = 0.0;
  return p;
}

}  // namespace

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

nav_msgs::msg::Path PlannerCore::computePath(
  const nav_msgs::msg::OccupancyGrid & map,
  double start_x, double start_y,
  double goal_x, double goal_y,
  const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = map.header.frame_id;

  CellIndex start = worldToGrid(map, start_x, start_y);
  CellIndex goal = worldToGrid(map, goal_x, goal_y);

  if (!isFree(map, start) || !isFree(map, goal)) {
    RCLCPP_WARN(logger_, "A*: start or goal cell is out of bounds or occupied, cannot plan");
    return path;
  }

  auto heuristic = [&goal](const CellIndex & cell) {
    double dx = cell.x - goal.x;
    double dy = cell.y - goal.y;
    return std::sqrt(dx * dx + dy * dy);
  };

  // Open set of nodes to be evaluated, ordered by f_score
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  // Closed set of nodes already evaluated
  std::unordered_set<CellIndex, CellIndexHash> closed_set;

  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start));

  static const std::vector<CellIndex> kNeighbors = {
    CellIndex(1, 0), CellIndex(-1, 0), CellIndex(0, 1), CellIndex(0, -1),
    CellIndex(1, 1), CellIndex(1, -1), CellIndex(-1, 1), CellIndex(-1, -1),
  };

  bool found = false;
  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal) {
      found = true;
      break;
    }

    if (closed_set.count(current)) {
      continue;
    }
    closed_set.insert(current);

    for (const auto & offset : kNeighbors) {
      CellIndex neighbor(current.x + offset.x, current.y + offset.y);
      if (!isFree(map, neighbor) || closed_set.count(neighbor)) {
        continue;
      }

      double step_cost = std::sqrt(static_cast<double>(offset.x * offset.x + offset.y * offset.y));
      double tentative_g = g_score[current] + step_cost;

      auto it = g_score.find(neighbor);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current;
        open_set.emplace(neighbor, tentative_g + heuristic(neighbor));
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A*: no path found from start to goal");
    return path;
  }

  // Expand nodes until the goal is reached, then walk came_from back to the start
  std::vector<CellIndex> cell_path;
  CellIndex cell = goal;
  while (!(cell == start)) {
    cell_path.push_back(cell);
    cell = came_from.at(cell);
  }
  cell_path.push_back(start);
  std::reverse(cell_path.begin(), cell_path.end());

  path.poses.reserve(cell_path.size());
  for (const auto & c : cell_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = gridToWorld(map, c);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

}
