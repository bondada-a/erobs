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
    MoveItLifecycleManager(
        rclcpp::Node* node,
        std::shared_ptr<mtc_pipeline::GripperConfigRegistry> registry,
        URToolInterface* tool_interface
    );

    ~MoveItLifecycleManager();

    // Launch MoveIt with gripper config (reuses existing if same gripper)
    bool launch_for_gripper(const std::string& gripper, const std::string& robot_ip);

    // Kill current MoveIt process (SIGTERM then SIGKILL if needed)
    void kill_current_process();

    std::string current_gripper() const;

private:
    rclcpp::Node* node_;
    std::shared_ptr<mtc_pipeline::GripperConfigRegistry> gripper_registry_;
    URToolInterface* tool_interface_;  // NOT owned

    std::string current_gripper_;
    pid_t moveit_pid_{0};

    // Fork and exec MoveIt launch file, returns child PID
    pid_t launch_moveit_process(
        const std::string& package,
        const std::string& launch_file,
        const std::string& robot_ip
    );
};

}
