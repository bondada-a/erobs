// Gripper utilities: derive MoveIt group names and SRDF states from gripper types.

#pragma once

#include <string>
#include <stdexcept>

namespace mtc_pipeline::gripper_utils {

/// @brief Get MoveIt planning group name for gripper type
inline std::string get_group_name(const std::string& type) {
    if (type.empty() || type == "none" || type == "pipettor") {
        return "";
    }
    return type + "_gripper";
}

/// @brief Get SRDF state name for gripper open/close action
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
