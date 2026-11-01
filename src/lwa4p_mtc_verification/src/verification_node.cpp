#include "lwa4p_mtc_verification/verification_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("lwa4p_mtc_verification", options);
  {
    lwa4p_mtc_verification::VerificationServer server(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  rclcpp::shutdown();
  return 0;
}
