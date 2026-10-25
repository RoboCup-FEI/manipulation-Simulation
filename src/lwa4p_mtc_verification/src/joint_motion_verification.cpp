#include "lwa4p_mtc_verification/joint_motion_verification.hpp"

#include <cmath>
#include <stdexcept>

namespace lwa4p_mtc_verification
{

bool has_reached_final_state(const JointPositions & initial, const JointPositions & measured)
{
  for (const auto & target : initial) {
    if (!std::isfinite(measured.at(target.first)) ||
      std::abs(measured.at(target.first) - target.second) > 0.035)
    {
      return false;
    }
  }
  return std::abs(measured.at("gripper_finger_left_joint") - 0.0425) < 0.002 &&
    std::abs(measured.at("gripper_finger_right_joint") - 0.0425) < 0.002;
}

void check_joint_motion(
  bool settled, const JointPositions & minimum, const JointPositions & maximum)
{
  if (!settled || maximum.at("arm_1_joint") - minimum.at("arm_1_joint") < 0.06) {
    throw std::runtime_error("MTC result not confirmed by measured arm motion");
  }
  for (const std::string name : {"gripper_finger_left_joint", "gripper_finger_right_joint"}) {
    if (minimum.at(name) > 0.02 || maximum.at(name) < 0.035) {
      throw std::runtime_error("Physical finger motion not observed: " + name);
    }
  }
}

}  // namespace lwa4p_mtc_verification
