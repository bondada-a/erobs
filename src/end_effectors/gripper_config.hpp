#pragma once

#include <string>

/**
 * @file gripper_config.hpp
 * @brief Centralized gripper configuration for MoveIt planning
 *
 * This header provides gripper-specific information needed for MTC planning,
 * including MoveIt group names and state names from SRDF files.
 *
 * When adding a new gripper:
 * 1. Add entry to get_gripper_config()
 * 2. Ensure SRDF defines the group and states
 * 3. Update this comment with the new gripper name
 *
 * Supported grippers: hande, epick, pipettor
 */

namespace gripper_config {

struct GripperInfo {
  std::string group;          // MoveIt planning group name from SRDF
  std::string release_state;  // State name when gripper is open/released
  std::string grasp_state;    // State name when gripper is closed/gripping
};

/**
 * @brief Get gripper configuration for MoveIt planning
 * @param gripper_type Gripper identifier (e.g., "hande", "epick", "pipettor")
 * @return GripperInfo with MoveIt group and state names
 */
inline GripperInfo get_gripper_config(const std::string& gripper_type) {
  if (gripper_type == "epick") {
    // EPick vacuum gripper: vacuum_off = released, vacuum_on = gripping
    return {"epick_gripper", "vacuum_off", "vacuum_on"};
  }

  if (gripper_type == "pipettor") {
    // Pipettor has no movable gripper joints (empty strings)
    return {"", "", ""};
  }

  // Default: Hand-E gripper
  // hande_open = released, hande_closed = gripping
  return {"hande_gripper", "hande_open", "hande_closed"};
}

}  // namespace gripper_config
