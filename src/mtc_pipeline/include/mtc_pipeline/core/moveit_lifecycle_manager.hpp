// MoveIt Lifecycle Manager: Launch/kill MoveIt processes based on gripper configuration
// Extracted from MTCOrchestratorActionServer for better separation of concerns.
// Manages the lifecycle of MoveIt move_group processes, including:
// - Forking and launching MoveIt with gripper-specific configurations
// - Gracefully killing processes (SIGTERM → SIGKILL)
// - Zombie process cleanup via SIGCHLD handler
// - Waiting for MoveIt planning service to be ready

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

namespace mtc_pipeline {
namespace core {

class MoveItLifecycleManager
{
public:
    MoveItLifecycleManager(
        rclcpp::Node* node,
        std::shared_ptr<mtc_pipeline::GripperConfigRegistry> registry,
        URToolInterface* tool_interface
    );

    ~MoveItLifecycleManager();

    // Launch MoveIt for a specific gripper configuration
    // If already running with same gripper, reuses the existing process
    // If different gripper, kills old process and launches new one
    // Returns true on success, false on failure
    bool launch_for_gripper(const std::string& gripper, const std::string& robot_ip);

    // Kill the current MoveIt process (if running)
    // Sends SIGTERM, waits 2s, then SIGKILL if still alive
    void kill_current_process();

    // Get the name of the currently loaded gripper
    // Returns empty string if no MoveIt process running
    std::string current_gripper() const;

private:
    rclcpp::Node* node_;
    std::shared_ptr<mtc_pipeline::GripperConfigRegistry> gripper_registry_;
    URToolInterface* tool_interface_;  // NOT owned

    std::string current_gripper_;
    pid_t moveit_pid_{0};

    // Helper: fork and exec MoveIt launch file
    // Returns PID of child process, or -1 on failure
    pid_t launch_moveit_process(
        const std::string& package,
        const std::string& launch_file,
        const std::string& robot_ip
    );
};

}  // namespace core
}  // namespace mtc_pipeline
