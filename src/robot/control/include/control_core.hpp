#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit ControlCore(const rclcpp::Logger& logger);

    // Straight-line distance between two points
    double computeDistance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) const;

    // Yaw angle (radians) extracted from a quaternion
    double extractYaw(const geometry_msgs::msg::Quaternion & quat) const;

    // First path pose at least lookahead_distance away from the robot; falls back to the final
    // pose if the entire remaining path is already closer than that
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
      const nav_msgs::msg::Path & path, const geometry_msgs::msg::Pose & robot_pose,
      double lookahead_distance) const;

    // Pure pursuit velocity command: constant forward speed, curvature steers toward target
    geometry_msgs::msg::Twist computeVelocity(
      const geometry_msgs::msg::Pose & robot_pose,
      const geometry_msgs::msg::PoseStamped & target,
      double linear_speed,
      double lookahead_distance) const;

  private:
    rclcpp::Logger logger_;
};

}

#endif
