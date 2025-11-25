// Gripper configuration: maps gripper types to MoveIt group/state names.
// Supported: hande, epick, pipettor

#pragma once

#include <string>

namespace gripper_config {

struct GripperInfo {
  std::string group;          // MoveIt planning group name from SRDF
  std::string release_state;  // State name when gripper is open/released
  std::string grasp_state;    // State name when gripper is closed/gripping
};

inline GripperInfo get_gripper_config(const std::string& gripper_type) {
  if (gripper_type == "epick") {
    return {"epick_gripper", "vacuum_off", "vacuum_on"};
  }

  if (gripper_type == "pipettor") {
    return {"", "", ""};  // No movable joints
  }

  // Default: Hand-E gripper
  return {"hande_gripper", "hande_open", "hande_closed"};
}

}  // namespace gripper_config
