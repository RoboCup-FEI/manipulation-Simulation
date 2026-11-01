#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "lwa4p_calibration_interfaces/srv/send_to_pose.hpp"
#include "lwa4p_calibration_poses/arm_motion_client.hpp"
#include "lwa4p_calibration_poses/joint_pose_file.hpp"

namespace lwa4p_calibration_poses
{

/**
 * @brief Teaches calibration poses.
 *
 * send_to_pose moves the arm to the requested joint positions.
 * save_pose appends the actual joint positions to the pose file.
 */
class PoseTeacherNode : public rclcpp::Node
{
public:
  PoseTeacherNode();

private:
  using SendToPose = lwa4p_calibration_interfaces::srv::SendToPose;
  using Trigger = std_srvs::srv::Trigger;

  void on_send_to_pose_requested(
    const SendToPose::Request::SharedPtr request,
    SendToPose::Response::SharedPtr response);

  void on_save_pose_requested(
    const Trigger::Request::SharedPtr request,
    Trigger::Response::SharedPtr response);

  ArmMotionClient arm_motion_client_;
  JointPoseFile pose_file_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<SendToPose>::SharedPtr send_to_pose_service_;
  rclcpp::Service<Trigger>::SharedPtr save_pose_service_;
};

}  // namespace lwa4p_calibration_poses
