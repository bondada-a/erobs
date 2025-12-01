// Gripper utilities: derive MoveIt group names and SRDF states from gripper types.

#pragma once

#include <string>
#include <stdexcept>

namespace mtc_pipeline::gripper_utils {

// Returns MoveIt planning group name (e.g., "hande_gripper").
// Static grippers (pipettor) return empty string.
inline std::string get_group_name(const std::string& type) {
    if (type.empty() || type == "none" || type == "pipettor") {
        return "";
    }
    return type + "_gripper";
}

// Returns SRDF state name for gripper action (e.g., "hande_open", "vacuum_on").
// Special case: epick uses vacuum_on/vacuum_off instead of open/closed.
inline std::string get_state_name(const std::string& type, bool open) {
    if (type == "epick") {
        return open ? "vacuum_off" : "vacuum_on";
    }

    if (type.empty() || type == "none") {
        throw std::invalid_argument("Cannot get state for gripper type: '" + type + "'");
    }

    return type + (open ? "_open" : "_closed");
}

}
