#include "lwa4p_mtc_verification/verification_server.hpp"

#include <exception>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace lwa4p_mtc_verification
{

VerificationServer::VerificationServer(rclcpp::Node::SharedPtr node)
: node_(std::move(node)), verification_(node_)
{
  report_path_ = node_->get_parameter("report_path").as_string();
  if (report_path_.empty()) {
    throw std::invalid_argument("report_path must name the verification JSON output");
  }
  service_ = node_->create_service<Trigger>("lwa4p/test_ptp",
    [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Trigger::Request> request)
    {
      on_test_requested(std::move(header), std::move(request));
    });
  worker_ = std::thread(&VerificationServer::worker_loop, this);
  RCLCPP_INFO(node_->get_logger(), "Ready: /lwa4p/test_ptp (simulation verification only)");
}

VerificationServer::~VerificationServer()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  requested_.notify_one();
  worker_.join();
}

void VerificationServer::on_test_requested(
  std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Trigger::Request>)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (busy_ || stopping_) {
    Trigger::Response response;
    response.success = false;
    response.message = stopping_ ? "STOPPING" : "BUSY";
    service_->send_response(*header, response);
    return;
  }
  busy_ = true;
  pending_request_ = std::move(header);
  requested_.notify_one();
}

void VerificationServer::worker_loop()
{
  while (true) {
    std::shared_ptr<rmw_request_id_t> header;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      requested_.wait(lock,
        [this]()
        {
          return stopping_ || pending_request_;
        });
      if (stopping_) {
        return;
      }
      header = std::move(pending_request_);
    }

    nlohmann::json report;
    try {
      report = verification_.run();
    } catch (const std::exception & error) {
      report = {{"success", false}, {"error", error.what()}};
    }
    std::ofstream output(report_path_);
    output << report.dump(2) << '\n';
    output.close();
    if (!output) {
      report = {{"success", false}, {"error", "Cannot write report: " + report_path_}};
    }
    Trigger::Response response;
    response.success = report.at("success").get<bool>();
    response.message = response.success ? "MTC motion verified: " + report_path_
                                        : report.at("error").get<std::string>();
    RCLCPP_INFO(node_->get_logger(), "%s", response.message.c_str());
    try {
      // Humble's deferred response keeps the single executor available during execution.
      service_->send_response(*header, response);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_->get_logger(), "Service response failed: %s", error.what());
    }
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = false;
  }
}

}  // namespace lwa4p_mtc_verification
