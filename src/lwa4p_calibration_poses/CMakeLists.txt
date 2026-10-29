cmake_minimum_required(VERSION 3.8)
project(lwa4p_calibration_poses)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(lwa4p_calibration_interfaces REQUIRED)

add_library(calibration_poses_logic STATIC
  src/joint_pose_file.cpp
  src/quintic_ptp_planner.cpp)
target_include_directories(calibration_poses_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(calibration_poses_logic PUBLIC cxx_std_17)

add_library(arm_motion_client STATIC
  src/arm_motion_client_ros.cpp)
target_link_libraries(arm_motion_client PUBLIC
  calibration_poses_logic
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${control_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${trajectory_msgs_TARGETS})

add_executable(pose_teacher_node
  src/pose_teacher_node.cpp
  src/pose_teacher_ros.cpp)
target_link_libraries(pose_teacher_node
  arm_motion_client
  ${std_srvs_TARGETS}
  ${lwa4p_calibration_interfaces_TARGETS})

add_executable(calibration_sequence_node
  src/calibration_sequence_node.cpp
  src/calibration_sequence_ros.cpp)
target_link_libraries(calibration_sequence_node
  arm_motion_client
  ${std_srvs_TARGETS})

install(TARGETS pose_teacher_node calibration_sequence_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_quintic_ptp_planner test/test_quintic_ptp_planner.cpp)
  target_link_libraries(test_quintic_ptp_planner calibration_poses_logic)

  ament_add_gtest(test_joint_pose_file test/test_joint_pose_file.cpp)
  target_link_libraries(test_joint_pose_file calibration_poses_logic)
endif()

ament_package()
