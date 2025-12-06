// Manages MoveIt process lifecycle for different gripper configurations.
// Handles launching, killing, and switching between gripper-specific MoveIt instances.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <string>
#include <memory>
#include <sys/types.h>

// Forward declarations
namespace mtc_pipeline {
    class GripperConfigRegistry;
    namespace core {
        class URToolInterface;
    }
}

namespace mtc_pipeline::core {

class MoveItLifecycleManager
{
public:
    /// @brief Construct MoveIt lifecycle manager with dependencies
    MoveItLifecycleManager(
        rclcpp::Node* node,
        std::shared_ptr<mtc_pipeline::GripperConfigRegistry> registry,
        URToolInterface* tool_interface
    );

    ~MoveItLifecycleManager();

    /// @brief Launch MoveIt for specified gripper configuration
    bool launch_for_gripper(const std::string& gripper, const std::string& robot_ip);

    /// @brief Kill current MoveIt process gracefully then forcefully
    void kill_current_process();

    /// @brief Get currently loaded gripper type
    std::string current_gripper() const;

private:
    rclcpp::Node* node_;
    std::shared_ptr<mtc_pipeline::GripperConfigRegistry> gripper_registry_;
    URToolInterface* tool_interface_;  // NOT owned

    std::string current_gripper_;
    pid_t moveit_pid_{0};

    /// @brief Fork and execute MoveIt launch file
    pid_t launch_moveit_process(
        const std::string& package,
        const std::string& launch_file,
        const std::string& robot_ip
    );
};

}
