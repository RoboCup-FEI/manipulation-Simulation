#pragma once

#include <vector>

namespace lwa4p_calibration_poses
{

struct TrajectoryPoint
{
  double time_from_start_s;
  std::vector<double> positions;
  std::vector<double> velocities;
};

/**
 * @brief Plans a synchronized point-to-point joint motion with quintic time scaling.
 *
 * All joints start and stop together with zero velocity and acceleration.
 * The duration is the shortest one that keeps every joint within the velocity
 * and acceleration limits.
 */
class QuinticPtpPlanner
{
public:
  /**
   * @param max_velocity joint velocity limit [rad/s]
   * @param max_acceleration joint acceleration limit [rad/s^2]
   * @param sample_period time step between trajectory points [s]
   * @throws std::invalid_argument if any value is not positive and finite
   */
  QuinticPtpPlanner(double max_velocity, double max_acceleration, double sample_period);

  /**
   * @brief Samples the motion at t = sample_period ... duration.
   *
   * The start point itself is not included: the trajectory executor prepends
   * its own actual position.
   * @throws std::invalid_argument if the sizes differ or a position is not finite
   */
  std::vector<TrajectoryPoint> plan(
    const std::vector<double> & start_positions,
    const std::vector<double> & goal_positions) const;

private:
  double compute_duration(double max_displacement) const;

  double max_velocity_;
  double max_acceleration_;
  double sample_period_;
};

}  // namespace lwa4p_calibration_poses
