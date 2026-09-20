#include <chrono>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode()
  : Node("control"),
    control_(robot::ControlCore(this->get_logger())),
    lookahead_distance_(1.0),  // Lookahead distance
    goal_tolerance_(0.1),      // Distance to consider the goal reached
    linear_speed_(0.5) {       // Constant forward speed
  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
    return;
  }

  const auto & robot_pose = robot_odom_->pose.pose;

  // Stop once the final goal has been reached
  double distance_to_goal = control_.computeDistance(robot_pose.position, current_path_->poses.back().pose.position);
  if (distance_to_goal < goal_tolerance_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  // Find the lookahead point
  auto lookahead_point = control_.findLookaheadPoint(*current_path_, robot_pose, lookahead_distance_);
  if (!lookahead_point) {
    return;  // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = control_.computeVelocity(robot_pose, *lookahead_point, linear_speed_, lookahead_distance_);

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
