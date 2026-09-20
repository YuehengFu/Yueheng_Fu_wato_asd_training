#include <cmath>

#include "control_core.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {}

double ControlCore::computeDistance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion & quat) const {
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::Pose & robot_pose,
  double lookahead_distance) const {
  if (path.poses.empty()) {
    return std::nullopt;
  }

  for (const auto & pose : path.poses) {
    if (computeDistance(robot_pose.position, pose.pose.position) >= lookahead_distance) {
      return pose;
    }
  }

  // Every remaining path point is closer than the lookahead distance; head for the final pose
  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const geometry_msgs::msg::Pose & robot_pose,
  const geometry_msgs::msg::PoseStamped & target,
  double linear_speed,
  double lookahead_distance) const {
  double robot_yaw = extractYaw(robot_pose.orientation);

  double dx = target.pose.position.x - robot_pose.position.x;
  double dy = target.pose.position.y - robot_pose.position.y;

  // Angle of the lookahead point relative to the robot's own heading, normalized to [-pi, pi]
  double alpha = std::atan2(dy, dx) - robot_yaw;
  while (alpha > M_PI) alpha -= 2 * M_PI;
  while (alpha < -M_PI) alpha += 2 * M_PI;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed;
  // Pure pursuit curvature law: omega = 2 * v * sin(alpha) / lookahead_distance
  cmd_vel.angular.z = 2.0 * linear_speed * std::sin(alpha) / lookahead_distance;
  return cmd_vel;
}

}
