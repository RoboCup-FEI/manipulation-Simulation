// Copyright 2026 Dmitrii Ognev
// Licensed under the MIT License. See LICENSE in the repository root.

#include "lwa4p_calibration_poses/calibration_sequence_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto calibration_sequence_node =
    std::make_shared<lwa4p_calibration_poses::CalibrationSequenceNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(calibration_sequence_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
