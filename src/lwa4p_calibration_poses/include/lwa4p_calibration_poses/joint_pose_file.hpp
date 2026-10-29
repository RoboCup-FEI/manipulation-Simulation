// Copyright 2026 Dmitrii Ognev
// Licensed under the MIT License. See LICENSE in the repository root.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace lwa4p_calibration_poses
{

inline constexpr char DEFAULT_POSE_FILE_PATH[] = "~/calibration_poses.txt";

/**
 * @brief Text file with one joint pose per line: whitespace-separated positions [rad].
 *
 * Empty lines and everything after '#' are ignored, so the file can be annotated
 * and reordered by hand.
 */
class JointPoseFile
{
public:
  /**
   * @param path file path, a leading "~/" is expanded to $HOME
   * @param joint_count number of positions every pose must contain
   * @throws std::invalid_argument if joint_count is zero or "~/" cannot be expanded
   */
  JointPoseFile(const std::string & path, std::size_t joint_count);

  /**
   * @brief Appends one pose as a new line, creating the file if it does not exist.
   * @throws std::invalid_argument if the pose size differs from joint_count
   * @throws std::runtime_error if the file cannot be written
   */
  void append(const std::vector<double> & positions) const;

  /**
   * @return all poses in file order
   * @throws std::runtime_error if the file cannot be read or a line is malformed
   */
  std::vector<std::vector<double>> read_all() const;

  const std::string & path() const;

private:
  /// @return empty vector for a line without positions (blank or comment only)
  std::vector<double> parse_line(const std::string & line, std::size_t line_number) const;

  std::string path_;
  std::size_t joint_count_;
};

/// @return positions with fixed precision separated by spaces, the same format as a file line
std::string format_joint_positions(const std::vector<double> & positions);

}  // namespace lwa4p_calibration_poses
