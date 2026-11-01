#pragma once

#include <map>
#include <string>

namespace lwa4p_mtc_verification
{

using JointPositions = std::map<std::string, double>;

/// Check the final arm configuration and the simulated WSG's open aperture.
bool has_reached_final_state(const JointPositions & initial, const JointPositions & measured);

/// Require observed arm and finger motion, rather than only successful action results.
void check_joint_motion(
  bool settled, const JointPositions & minimum, const JointPositions & maximum);

}  // namespace lwa4p_mtc_verification
