#pragma once

#include <chrono>
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "lwa4p_calibration_poses/quintic_ptp_planner.hpp"

namespace lwa4p_calibration_poses
{

/**
 * @brief ROS helper shared by the pose teacher and the calibration sequence.
 *
 * Tracks the actual arm joint positions from joint_states and moves the arm to a joint goal
 * through a FollowJointTrajectory action.
 * Declares its own parameters (joint names, action name, motion limits) on the given node.
 * move_to() blocks until the motion has finished, so the node must be spun by
 * a multi-threaded executor: the subscription and the action client run in their own
 * callback group and keep being served while a service callback waits.
 */
class ArmMotionClient
{
public:
  /// @throws std::invalid_argument if a parameter value is invalid
  explicit ArmMotionClient(rclcpp::Node * node);

  std::size_t joint_count() const;

  /**
   * @return actual positions [rad] ordered as the joint_names parameter
   * @throws std::runtime_error if no joint_states message with all arm joints was received
   */
  std::vector<double> get_current_positions() const;

  /**
   * @brief Moves the arm to the goal and waits until the trajectory has finished.
   * @param goal_positions [rad] ordered as the joint_names parameter
   * @throws std::invalid_argument if the goal is malformed
   * @throws std::runtime_error if the goal is rejected, fails or times out
   */
  void move_to(const std::vector<double> & goal_positions);

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  void on_joint_state_received(sensor_msgs::msg::JointState::ConstSharedPtr joint_state);

  /// @return std::nullopt if the message does not contain every arm joint (e.g. gripper states)
  std::optional<std::vector<double>> extract_arm_positions(
    const sensor_msgs::msg::JointState & joint_state) const;

  FollowJointTrajectory::Goal build_goal(const std::vector<TrajectoryPoint> & trajectory) const;
  GoalHandle::SharedPtr send_goal(const FollowJointTrajectory::Goal & goal);
  void wait_for_result(const GoalHandle::SharedPtr & goal_handle, double trajectory_duration_s);

  rclcpp::Logger logger_;
  std::vector<std::string> joint_names_;
  std::string trajectory_action_name_;
  QuinticPtpPlanner planner_;
  double motion_timeout_margin_s_;

  rclcpp::CallbackGroup::SharedPtr motion_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;

  mutable std::mutex current_positions_mutex_;
  std::optional<std::vector<double>> current_positions_;
};

}  // namespace lwa4p_calibration_poses
