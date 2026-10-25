#pragma once

#include <condition_variable>
#include <map>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <nlohmann/json.hpp>

namespace lwa4p_mtc_verification
{

/// Integrate MTC execution with measured ROS joint feedback.
class ExecutionVerification
{
public:
  explicit ExecutionVerification(rclcpp::Node::SharedPtr node);
  nlohmann::json run();

private:
  void on_joint_state(const sensor_msgs::msg::JointState & message);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::mutex mutex_;
  std::condition_variable updated_;
  std::map<std::string, double> positions_;
  std::map<std::string, double> minimum_;
  std::map<std::string, double> maximum_;
};

}  // namespace lwa4p_mtc_verification
