// Copyright 2026 Dmitrii Ognev
// Licensed under the MIT License. See LICENSE in the repository root.

#include "lwa4p_calibration_poses/joint_pose_file.hpp"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace lwa4p_calibration_poses
{

namespace
{

constexpr int POSITION_PRECISION = 6;

std::string expand_home_directory(const std::string & path)
{
  if (path.compare(0, 2, "~/") != 0) {
    return path;
  }
  const char * home_directory = std::getenv("HOME");
  if (home_directory == nullptr) {
    throw std::invalid_argument("Cannot expand '" + path + "': HOME is not set");
  }
  return std::string(home_directory) + path.substr(1);
}

}  // namespace

JointPoseFile::JointPoseFile(const std::string & path, std::size_t joint_count)
: path_(expand_home_directory(path)),
  joint_count_(joint_count)
{
  if (joint_count_ == 0) {
    throw std::invalid_argument("Joint count must be positive");
  }
}

void JointPoseFile::append(const std::vector<double> & positions) const
{
  if (positions.size() != joint_count_) {
    throw std::invalid_argument(
      "Pose has " + std::to_string(positions.size()) + " joint positions, expected " +
      std::to_string(joint_count_));
  }

  std::ofstream file(path_, std::ios::app);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open pose file '" + path_ + "' for writing");
  }
  file << format_joint_positions(positions) << '\n';
  if (!file) {
    throw std::runtime_error("Failed to write to pose file '" + path_ + "'");
  }
}

std::vector<std::vector<double>> JointPoseFile::read_all() const
{
  std::ifstream file(path_);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open pose file '" + path_ + "'");
  }

  std::vector<std::vector<double>> poses;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    auto positions = parse_line(line, line_number);
    if (!positions.empty()) {
      poses.push_back(std::move(positions));
    }
  }
  return poses;
}

const std::string & JointPoseFile::path() const
{
  return path_;
}

std::vector<double> JointPoseFile::parse_line(
  const std::string & line, std::size_t line_number) const
{
  const std::string error_prefix = path_ + ":" + std::to_string(line_number) + ": ";
  std::istringstream stream(line.substr(0, line.find('#')));

  std::vector<double> positions;
  double position = 0.0;
  while (stream >> position) {
    positions.push_back(position);
  }
  if (!stream.eof()) {
    throw std::runtime_error(error_prefix + "not a number");
  }
  if (!positions.empty() && positions.size() != joint_count_) {
    throw std::runtime_error(
      error_prefix + std::to_string(positions.size()) + " joint positions, expected " +
      std::to_string(joint_count_));
  }
  return positions;
}

std::string format_joint_positions(const std::vector<double> & positions)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(POSITION_PRECISION);
  for (std::size_t joint = 0; joint < positions.size(); ++joint) {
    if (joint > 0) {
      stream << ' ';
    }
    stream << positions[joint];
  }
  return stream.str();
}

}  // namespace lwa4p_calibration_poses
