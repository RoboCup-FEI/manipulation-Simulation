#pragma once

#include <chrono>
#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "lwa4p_calibration_poses/arm_motion_client.hpp"
#include "lwa4p_calibration_poses/joint_pose_file.hpp"

namespace lwa4p_calibration_poses
{

/**
 * @brief Replays the taught calibration poses.
 *
 * run_calibration_sequence moves the arm through every pose of the pose file in order
 * and holds still at each one for settle_time_s, so the camera captures a static robot.
 */
class CalibrationSequenceNode : public rclcpp::Node
{
public:
  /// @throws std::invalid_argument if settle_time_s is negative
  CalibrationSequenceNode();

private:
  using Trigger = std_srvs::srv::Trigger;

  void on_run_calibration_sequence_requested(
    const Trigger::Request::SharedPtr request,
    Trigger::Response::SharedPtr response);

  /// @throws std::runtime_error if the wait is interrupted by shutdown
  void hold_pose(std::size_t pose_number, std::size_t pose_count) const;

  ArmMotionClient arm_motion_client_;
  JointPoseFile pose_file_;
  double settle_time_s_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<Trigger>::SharedPtr run_calibration_sequence_service_;
};

}  // namespace lwa4p_calibration_poses
