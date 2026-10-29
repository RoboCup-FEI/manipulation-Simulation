// Copyright 2026 Dmitrii Ognev
// Licensed under the MIT License. See LICENSE in the repository root.

#include "lwa4p_calibration_poses/pose_teacher_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto pose_teacher_node = std::make_shared<lwa4p_calibration_poses::PoseTeacherNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pose_teacher_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
