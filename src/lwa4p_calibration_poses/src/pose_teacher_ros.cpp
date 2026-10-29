// Copyright 2026 Dmitrii Ognev
// Licensed under the MIT License. See LICENSE in the repository root.

#include "lwa4p_calibration_poses/pose_teacher_node.hpp"

#include <exception>
#include <string>

namespace lwa4p_calibration_poses
{

PoseTeacherNode::PoseTeacherNode()
: Node("pose_teacher"),
  arm_motion_client_(this),
  pose_file_(
    declare_parameter<std::string>("pose_file_path", DEFAULT_POSE_FILE_PATH),
    arm_motion_client_.joint_count())
{
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  send_to_pose_service_ = create_service<SendToPose>(
    "send_to_pose",
    std::bind(
      &PoseTeacherNode::on_send_to_pose_requested, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_callback_group_);

  save_pose_service_ = create_service<Trigger>(
    "save_pose",
    std::bind(
      &PoseTeacherNode::on_save_pose_requested, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_callback_group_);

  RCLCPP_INFO(
    get_logger(), "Pose teacher ready, poses are saved to '%s'", pose_file_.path().c_str());
}

void PoseTeacherNode::on_send_to_pose_requested(
  const SendToPose::Request::SharedPtr request,
  SendToPose::Response::SharedPtr response)
{
  try {
    arm_motion_client_.move_to(request->positions);
    response->success = true;
    response->message = "Pose reached";
  } catch (const std::exception & error) {
    response->success = false;
    response->message = error.what();
    RCLCPP_ERROR(get_logger(), "send_to_pose failed: %s", error.what());
  }
}

void PoseTeacherNode::on_save_pose_requested(
  const Trigger::Request::SharedPtr,
  Trigger::Response::SharedPtr response)
{
  try {
    const auto current_positions = arm_motion_client_.get_current_positions();
    pose_file_.append(current_positions);
    response->success = true;
    response->message =
      "Saved [" + format_joint_positions(current_positions) + "] to " + pose_file_.path();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } catch (const std::exception & error) {
    response->success = false;
    response->message = error.what();
    RCLCPP_ERROR(get_logger(), "save_pose failed: %s", error.what());
  }
}

}  // namespace lwa4p_calibration_poses
