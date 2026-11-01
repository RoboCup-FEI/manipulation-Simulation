#include "lwa4p_calibration_poses/calibration_sequence_node.hpp"

#include <exception>
#include <stdexcept>
#include <string>

namespace lwa4p_calibration_poses
{

CalibrationSequenceNode::CalibrationSequenceNode()
: Node("calibration_sequence"),
  arm_motion_client_(this),
  pose_file_(
    declare_parameter<std::string>("pose_file_path", DEFAULT_POSE_FILE_PATH),
    arm_motion_client_.joint_count()),
  settle_time_s_(declare_parameter<double>("settle_time_s", 3.0))
{
  if (settle_time_s_ < 0.0) {
    throw std::invalid_argument("Parameter 'settle_time_s' must not be negative");
  }

  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  run_calibration_sequence_service_ = create_service<Trigger>(
    "run_calibration_sequence",
    std::bind(
      &CalibrationSequenceNode::on_run_calibration_sequence_requested, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_callback_group_);

  RCLCPP_INFO(
    get_logger(), "Calibration sequence ready, poses are read from '%s'",
    pose_file_.path().c_str());
}

void CalibrationSequenceNode::on_run_calibration_sequence_requested(
  const Trigger::Request::SharedPtr,
  Trigger::Response::SharedPtr response)
{
  std::size_t reached_pose_count = 0;
  try {
    const auto poses = pose_file_.read_all();
    if (poses.empty()) {
      throw std::runtime_error("Pose file '" + pose_file_.path() + "' contains no poses");
    }

    for (const auto & pose : poses) {
      RCLCPP_INFO(
        get_logger(), "Pose %zu/%zu: [%s]",
        reached_pose_count + 1, poses.size(), format_joint_positions(pose).c_str());
      arm_motion_client_.move_to(pose);
      ++reached_pose_count;
      hold_pose(reached_pose_count, poses.size());
    }

    response->success = true;
    response->message =
      "Calibration sequence finished, " + std::to_string(reached_pose_count) + " poses reached";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } catch (const std::exception & error) {
    response->success = false;
    response->message =
      "Calibration sequence stopped after " + std::to_string(reached_pose_count) +
      " reached poses: " + error.what();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }
}

void CalibrationSequenceNode::hold_pose(std::size_t pose_number, std::size_t pose_count) const
{
  RCLCPP_INFO(
    get_logger(), "Pose %zu/%zu reached, holding still for %.1f s",
    pose_number, pose_count, settle_time_s_);

  const auto settle_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(settle_time_s_));
  if (!rclcpp::sleep_for(settle_time)) {
    throw std::runtime_error("Interrupted by shutdown");
  }
}

}  // namespace lwa4p_calibration_poses
