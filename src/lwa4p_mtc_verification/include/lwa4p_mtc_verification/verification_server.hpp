#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "lwa4p_mtc_verification/execution_verification.hpp"

namespace lwa4p_mtc_verification
{

/// Expose the simulation check as a service, executed by one worker with BUSY admission.
class VerificationServer
{
public:
  explicit VerificationServer(rclcpp::Node::SharedPtr node);
  ~VerificationServer();

private:
  using Trigger = std_srvs::srv::Trigger;

  void on_test_requested(
    std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Trigger::Request> request);
  void worker_loop();

  rclcpp::Node::SharedPtr node_;
  ExecutionVerification verification_;
  rclcpp::Service<Trigger>::SharedPtr service_;
  std::string report_path_;
  std::mutex mutex_;
  std::condition_variable requested_;
  std::shared_ptr<rmw_request_id_t> pending_request_;
  bool busy_ = false;
  bool stopping_ = false;
  std::thread worker_;
};

}  // namespace lwa4p_mtc_verification
