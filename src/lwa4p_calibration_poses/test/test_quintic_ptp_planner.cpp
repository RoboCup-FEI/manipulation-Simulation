#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "lwa4p_calibration_poses/quintic_ptp_planner.hpp"

using lwa4p_calibration_poses::QuinticPtpPlanner;

namespace
{

constexpr double MAX_VELOCITY = 0.2;
constexpr double MAX_ACCELERATION = 0.2;
constexpr double SAMPLE_PERIOD = 0.01;
constexpr double TOLERANCE = 1e-9;

const std::vector<double> START = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const std::vector<double> GOAL = {1.0, -0.5, 0.2, 0.0, 0.3, -1.2};

}  // namespace

TEST(QuinticPtpPlannerTest, EndsAtGoalAtRest)
{
  const QuinticPtpPlanner planner(MAX_VELOCITY, MAX_ACCELERATION, SAMPLE_PERIOD);
  const auto trajectory = planner.plan(START, GOAL);

  ASSERT_FALSE(trajectory.empty());
  for (std::size_t joint = 0; joint < GOAL.size(); ++joint) {
    EXPECT_NEAR(trajectory.back().positions[joint], GOAL[joint], TOLERANCE);
    EXPECT_NEAR(trajectory.back().velocities[joint], 0.0, TOLERANCE);
  }
}

TEST(QuinticPtpPlannerTest, StartsAfterTimeZero)
{
  const QuinticPtpPlanner planner(MAX_VELOCITY, MAX_ACCELERATION, SAMPLE_PERIOD);
  const auto trajectory = planner.plan(START, GOAL);

  EXPECT_GT(trajectory.front().time_from_start_s, 0.0);
}

TEST(QuinticPtpPlannerTest, RespectsVelocityAndAccelerationLimits)
{
  const QuinticPtpPlanner planner(MAX_VELOCITY, MAX_ACCELERATION, SAMPLE_PERIOD);
  const auto trajectory = planner.plan(START, GOAL);

  for (std::size_t sample = 1; sample < trajectory.size(); ++sample) {
    const double time_step =
      trajectory[sample].time_from_start_s - trajectory[sample - 1].time_from_start_s;
    for (std::size_t joint = 0; joint < GOAL.size(); ++joint) {
      const double velocity = trajectory[sample].velocities[joint];
      const double acceleration =
        (velocity - trajectory[sample - 1].velocities[joint]) / time_step;
      EXPECT_LE(std::abs(velocity), MAX_VELOCITY + TOLERANCE);
      EXPECT_LE(std::abs(acceleration), MAX_ACCELERATION * 1.01);
    }
  }
}

TEST(QuinticPtpPlannerTest, ZeroDisplacementLastsOneSamplePeriod)
{
  const QuinticPtpPlanner planner(MAX_VELOCITY, MAX_ACCELERATION, SAMPLE_PERIOD);
  const auto trajectory = planner.plan(GOAL, GOAL);

  ASSERT_EQ(trajectory.size(), 1u);
  EXPECT_NEAR(trajectory.front().time_from_start_s, SAMPLE_PERIOD, TOLERANCE);
}

TEST(QuinticPtpPlannerTest, RejectsInvalidLimits)
{
  EXPECT_THROW(QuinticPtpPlanner(0.0, MAX_ACCELERATION, SAMPLE_PERIOD), std::invalid_argument);
  EXPECT_THROW(QuinticPtpPlanner(MAX_VELOCITY, -1.0, SAMPLE_PERIOD), std::invalid_argument);
  EXPECT_THROW(
    QuinticPtpPlanner(MAX_VELOCITY, MAX_ACCELERATION, std::nan("")), std::invalid_argument);
}

TEST(QuinticPtpPlannerTest, RejectsMalformedGoal)
{
  const QuinticPtpPlanner planner(MAX_VELOCITY, MAX_ACCELERATION, SAMPLE_PERIOD);
  const std::vector<double> short_goal = {1.0, 2.0};
  std::vector<double> infinite_goal = GOAL;
  infinite_goal[2] = std::numeric_limits<double>::infinity();

  EXPECT_THROW(planner.plan(START, short_goal), std::invalid_argument);
  EXPECT_THROW(planner.plan(START, infinite_goal), std::invalid_argument);
}
