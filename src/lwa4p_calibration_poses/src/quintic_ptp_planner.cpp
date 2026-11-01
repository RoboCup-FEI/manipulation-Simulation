#include "lwa4p_calibration_poses/quintic_ptp_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace lwa4p_calibration_poses
{

namespace
{

// Peak first and second derivatives of s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5 on [0, 1]
constexpr double QUINTIC_PEAK_VELOCITY_FACTOR = 1.875;
constexpr double QUINTIC_PEAK_ACCELERATION_FACTOR = 5.773502691896258;

double quintic_position_scale(double tau)
{
  return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
}

double quintic_velocity_scale(double tau)
{
  return 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau);
}

void check_positive(double value, const std::string & name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " must be positive, got " + std::to_string(value));
  }
}

void check_finite(const std::vector<double> & positions, const std::string & name)
{
  for (const double position : positions) {
    if (!std::isfinite(position)) {
      throw std::invalid_argument(name + " contains a non-finite value");
    }
  }
}

TrajectoryPoint make_point(
  const std::vector<double> & start_positions,
  const std::vector<double> & displacements,
  double duration,
  double tau)
{
  const double position_scale = quintic_position_scale(tau);
  const double velocity_scale = quintic_velocity_scale(tau) / duration;

  TrajectoryPoint point;
  point.time_from_start_s = tau * duration;
  for (std::size_t joint = 0; joint < displacements.size(); ++joint) {
    point.positions.push_back(start_positions[joint] + position_scale * displacements[joint]);
    point.velocities.push_back(velocity_scale * displacements[joint]);
  }
  return point;
}

}  // namespace

QuinticPtpPlanner::QuinticPtpPlanner(
  double max_velocity, double max_acceleration, double sample_period)
: max_velocity_(max_velocity),
  max_acceleration_(max_acceleration),
  sample_period_(sample_period)
{
  check_positive(max_velocity_, "max_velocity");
  check_positive(max_acceleration_, "max_acceleration");
  check_positive(sample_period_, "sample_period");
}

std::vector<TrajectoryPoint> QuinticPtpPlanner::plan(
  const std::vector<double> & start_positions,
  const std::vector<double> & goal_positions) const
{
  if (start_positions.size() != goal_positions.size()) {
    throw std::invalid_argument(
      "Goal has " + std::to_string(goal_positions.size()) + " joint positions, expected " +
      std::to_string(start_positions.size()));
  }
  check_finite(start_positions, "Start position");
  check_finite(goal_positions, "Goal position");

  std::vector<double> displacements;
  double max_displacement = 0.0;
  for (std::size_t joint = 0; joint < start_positions.size(); ++joint) {
    displacements.push_back(goal_positions[joint] - start_positions[joint]);
    max_displacement = std::max(max_displacement, std::abs(displacements.back()));
  }

  const double duration = compute_duration(max_displacement);
  const auto sample_count = static_cast<std::size_t>(std::ceil(duration / sample_period_));

  std::vector<TrajectoryPoint> trajectory;
  trajectory.reserve(sample_count);
  for (std::size_t sample = 1; sample <= sample_count; ++sample) {
    const double tau = static_cast<double>(sample) / static_cast<double>(sample_count);
    trajectory.push_back(make_point(start_positions, displacements, duration, tau));
  }
  return trajectory;
}

double QuinticPtpPlanner::compute_duration(double max_displacement) const
{
  const double velocity_limited_duration =
    QUINTIC_PEAK_VELOCITY_FACTOR * max_displacement / max_velocity_;
  const double acceleration_limited_duration =
    std::sqrt(QUINTIC_PEAK_ACCELERATION_FACTOR * max_displacement / max_acceleration_);
  return std::max({velocity_limited_duration, acceleration_limited_duration, sample_period_});
}

}  // namespace lwa4p_calibration_poses
