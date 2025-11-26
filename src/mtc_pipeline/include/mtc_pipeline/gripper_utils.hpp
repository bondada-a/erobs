// Gripper utility functions for MTC pipeline
// Provides shared helpers for deriving MoveIt group names and SRDF states from gripper types

#pragma once

#include <string>
#include <stdexcept>

namespace mtc_pipeline {
namespace gripper_utils {

/**
 * @brief Derives MoveIt group name from gripper type
 *
 * Maps gripper type strings to their corresponding MoveIt planning groups.
 * Grippers without movable joints (e.g., "pipettor") return empty string.
 *
 * @param type Gripper type identifier (e.g., "hande", "epick", "pipettor")
 * @return MoveIt group name (e.g., "hande_gripper") or empty string for static grippers
 *
 * @note This function consolidates logic previously duplicated across:
 *       - pick_place_stages.cpp
 *       - vision_pick_place_stages.cpp
 *       - end_effector_stages.cpp
 */
inline std::string get_group_name(const std::string& type) {
    // Static grippers have no movable joints, so no planning group
    if (type.empty() || type == "none" || type == "pipettor") {
        return "";
    }
    // Standard naming convention: <type>_gripper
    return type + "_gripper";
}

/**
 * @brief Derives SRDF state name for gripper action
 *
 * Maps gripper type and desired state (open/closed) to the corresponding
 * SRDF named state defined in the robot's semantic description.
 *
 * @param type Gripper type identifier
 * @param open True for open/released state, false for closed/activated state
 * @return SRDF state name (e.g., "hande_open", "vacuum_on")
 * @throws std::invalid_argument if type is empty or "none"
 *
 * @note Special cases:
 *       - "epick": Uses "vacuum_on"/"vacuum_off" instead of open/closed
 *       - Other grippers: Uses "<type>_open" / "<type>_closed"
 */
inline std::string get_state_name(const std::string& type, bool open) {
    // Special case: epick vacuum gripper uses on/off terminology
    if (type == "epick") {
        return open ? "vacuum_off" : "vacuum_on";
    }

    // Validate that type is actionable
    if (type.empty() || type == "none") {
        throw std::invalid_argument("Cannot get state for gripper type: '" + type + "'");
    }

    // Standard naming: <type>_open or <type>_closed
    return type + (open ? "_open" : "_closed");
}

}  // namespace gripper_utils
}  // namespace mtc_pipeline
