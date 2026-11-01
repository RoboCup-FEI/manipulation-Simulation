#include "lwa4p_calibration_poses/arm_motion_client.hpp"

#include <algorithm>
#include <future>
#include <stdexcept>
#include <utility>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "lwa4p_calibration_poses/joint_pose_file.hpp"

namespace lwa4p_calibration_poses
{

namespace
{

constexpr auto SERVER_RESPONSE_TIMEOUT = std::chrono::seconds(5);

const std::vector<std::string> DEFAULT_JOINT_NAMES = {
  "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint"};

std::string to_string(rclcpp_action::ResultCode result_code)
{
  switch (result_code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "succeeded";
    case rclcpp_action::ResultCode::ABORTED:
      return "aborted";
    case rclcpp_action::ResultCode::CANCELED:
      return "canceled";
    default:
      return "unknown";
  }
}

}  // namespace

ArmMotionClient::ArmMotionClient(rclcpp::Node * node)
: logger_(node->get_logger()),
  joint_names_(
    node->declare_parameter<std::vector<std::string>>("joint_names", DEFAULT_JOINT_NAMES)),
  trajectory_action_name_(
    node->declare_parameter<std::string>(
      "trajectory_action_name", "arm_controller/follow_joint_trajectory")),
  planner_(
    node->declare_parameter<double>("max_velocity_rad_s", 0.2),
    node->declare_parameter<double>("max_acceleration_rad_s2", 0.2),
    node->declare_parameter<double>("trajectory_sample_period_s", 0.1)),
  motion_timeout_margin_s_(node->declare_parameter<double>("motion_timeout_margin_s", 10.0))
{
  if (joint_names_.empty()) {
    throw std::invalid_argument("Parameter 'joint_names' must not be empty");
  }

  motion_callback_group_ =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = motion_callback_group_;
  joint_state_subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(),
    std::bind(&ArmMotionClient::on_joint_state_received, this, std::placeholders::_1),
    subscription_options);

  trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    node, trajectory_action_name_, motion_callback_group_);
}

std::size_t ArmMotionClient::joint_count() const
{
  return joint_names_.size();
}

std::vector<double> ArmMotionClient::get_current_positions() const
{
  std::lock_guard<std::mutex> lock(current_positions_mutex_);
  if (!current_positions_) {
    throw std::runtime_error("No joint_states received yet for all arm joints");
  }
  return *current_positions_;
}

void ArmMotionClient::move_to(const std::vector<double> & goal_positions)
{
  const auto trajectory = planner_.plan(get_current_positions(), goal_positions);
  const double trajectory_duration_s = trajectory.back().time_from_start_s;

  RCLCPP_INFO(
    logger_, "Moving to [%s] in %.1f s",
    format_joint_positions(goal_positions).c_str(), trajectory_duration_s);

  const auto goal_handle = send_goal(build_goal(trajectory));
  wait_for_result(goal_handle, trajectory_duration_s);
}

void ArmMotionClient::on_joint_state_received(
  sensor_msgs::msg::JointState::ConstSharedPtr joint_state)
{
  auto arm_positions = extract_arm_positions(*joint_state);
  if (!arm_positions) {
    return;
  }
  std::lock_guard<std::mutex> lock(current_positions_mutex_);
  current_positions_ = std::move(arm_positions);
}

std::optional<std::vector<double>> ArmMotionClient::extract_arm_positions(
  const sensor_msgs::msg::JointState & joint_state) const
{
  std::vector<double> arm_positions;
  for (const auto & joint_name : joint_names_) {
    const auto name_iterator =
      std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
    const auto index = static_cast<std::size_t>(name_iterator - joint_state.name.begin());
    if (index >= joint_state.position.size()) {
      return std::nullopt;
    }
    arm_positions.push_back(joint_state.position[index]);
  }
  return arm_positions;
}

ArmMotionClient::FollowJointTrajectory::Goal ArmMotionClient::build_goal(
  const std::vector<TrajectoryPoint> & trajectory) const
{
  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = joint_names_;
  for (const auto & point : trajectory) {
    trajectory_msgs::msg::JointTrajectoryPoint message_point;
    message_point.positions = point.positions;
    message_point.velocities = point.velocities;
    message_point.time_from_start = rclcpp::Duration::from_seconds(point.time_from_start_s);
    goal.trajectory.points.push_back(std::move(message_point));
  }
  return goal;
}

ArmMotionClient::GoalHandle::SharedPtr ArmMotionClient::send_goal(
  const FollowJointTrajectory::Goal & goal)
{
  if (!trajectory_action_client_->wait_for_action_server(SERVER_RESPONSE_TIMEOUT)) {
    throw std::runtime_error(
      "Action server '" + trajectory_action_name_ + "' is not available (is the driver running?)");
  }

  auto goal_handle_future = trajectory_action_client_->async_send_goal(goal);
  if (goal_handle_future.wait_for(SERVER_RESPONSE_TIMEOUT) != std::future_status::ready) {
    throw std::runtime_error(
      "No response to the trajectory goal from '" + trajectory_action_name_ + "'");
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    throw std::runtime_error(
      "Trajectory goal rejected by '" + trajectory_action_name_ +
      "' (are the drives enabled? see the driver log)");
  }
  return goal_handle;
}

void ArmMotionClient::wait_for_result(
  const GoalHandle::SharedPtr & goal_handle, double trajectory_duration_s)
{
  auto result_future = trajectory_action_client_->async_get_result(goal_handle);
  const auto result_timeout =
    std::chrono::duration<double>(trajectory_duration_s + motion_timeout_margin_s_);

  if (result_future.wait_for(result_timeout) != std::future_status::ready) {
    trajectory_action_client_->async_cancel_goal(goal_handle);
    throw std::runtime_error("Trajectory did not finish in time, goal canceled");
  }

  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    const std::string error_string =
      wrapped_result.result ? wrapped_result.result->error_string : "";
    throw std::runtime_error(
      "Trajectory " + to_string(wrapped_result.code) + ": " + error_string);
  }
}

}  // namespace lwa4p_calibration_poses
