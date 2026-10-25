cmake_minimum_required(VERSION 3.16)
project(lwa4p_mtc_verification)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(verify_mtc
  src/verification_node.cpp
  src/verification_server_ros.cpp
  src/execution_verification.cpp
  src/joint_motion_verification.cpp)
target_include_directories(verify_mtc PRIVATE include)
target_compile_features(verify_mtc PRIVATE cxx_std_17)
target_compile_options(verify_mtc PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(verify_mtc rclcpp sensor_msgs std_srvs moveit_task_constructor_core)
target_link_libraries(verify_mtc nlohmann_json::nlohmann_json)
install(TARGETS verify_mtc DESTINATION lib/${PROJECT_NAME})
ament_package()
