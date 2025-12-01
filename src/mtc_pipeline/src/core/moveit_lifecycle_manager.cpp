/**
 * MoveIt Lifecycle Manager Implementation
 *
 * Extracted from MTCOrchestratorActionServer:
 * - Lines 26-34: SIGCHLD handler
 * - Lines 295-373: initialize_moveit_stack()
 * - Lines 375-409: launch_moveit_process()
 * - Lines 411-433: kill_moveit_process()
 *
 * All logic preserved exactly as-is for behavior compatibility.
 */

#include "mtc_pipeline/core/moveit_lifecycle_manager.hpp"
#include "mtc_pipeline/core/ur_tool_interface.hpp"
#include "mtc_pipeline/gripper_config_registry.hpp"
#include "mtc_pipeline/obstacle_loader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <csignal>  // For signal(), SIGCHLD
#include <sys/wait.h>
#include <unistd.h>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

namespace mtc_pipeline {
namespace core {

// ============================================================================
// ZOMBIE PROCESS CLEANUP
// ============================================================================

// SIGCHLD handler: automatically reaps zombie child processes
// Called by kernel when any child process exits
// (EXACT copy from orchestrator lines 26-34)
static void sigchld_handler(int sig) {
    (void)sig;  // Unused parameter
    int saved_errno = errno;  // Save errno (signal handlers should be reentrant)

    // Reap all dead children (WNOHANG = don't block)
    while (waitpid(-1, nullptr, WNOHANG) > 0);

    errno = saved_errno;  // Restore errno
}

// ============================================================================
// CONSTRUCTION & DESTRUCTION
// ============================================================================

MoveItLifecycleManager::MoveItLifecycleManager(
    rclcpp::Node* node,
    std::shared_ptr<mtc_pipeline::GripperConfigRegistry> registry,
    URToolInterface* tool_interface)
    : node_(node),
      gripper_registry_(registry),
      tool_interface_(tool_interface),
      moveit_pid_(0)
{
    // Install SIGCHLD handler for automatic zombie cleanup
    signal(SIGCHLD, sigchld_handler);
}

MoveItLifecycleManager::~MoveItLifecycleManager()
{
    kill_current_process();
}

// ============================================================================
// PUBLIC INTERFACE
// ============================================================================

std::string MoveItLifecycleManager::current_gripper() const
{
    return current_gripper_.empty() ? "none" : current_gripper_;
}

bool MoveItLifecycleManager::launch_for_gripper(
    const std::string& gripper,
    const std::string& robot_ip)
{
    // (EXACT copy from orchestrator lines 295-373: initialize_moveit_stack)

    // Reuse existing MoveIt if same gripper
    if (moveit_pid_ > 0 && current_gripper_ == gripper) {
        RCLCPP_INFO(node_->get_logger(), "MoveIt already running for %s, reusing", gripper.c_str());
        return true;
    }

    // Kill existing MoveIt if different gripper
    if (moveit_pid_ > 0) {
        RCLCPP_INFO(node_->get_logger(), "Switching gripper: %s → %s",
                    current_gripper_.c_str(), gripper.c_str());
        kill_current_process();
    }

    // Get gripper configuration from registry
    auto config = gripper_registry_->get_config(gripper);
    if (!config) {
        // Build list of available grippers for error message
        std::string available_grippers;
        for (const auto& g : gripper_registry_->available_grippers()) {
            available_grippers += g + " ";
        }
        RCLCPP_ERROR(node_->get_logger(),
                     "Unknown gripper type: %s (available: %s)",
                     gripper.c_str(), available_grippers.c_str());
        return false;
    }

    // Step 1: Set tool voltage (must happen BEFORE MoveIt launches)
    RCLCPP_INFO(node_->get_logger(), "Setting tool voltage: %dV", config->tool_voltage);
    if (!tool_interface_->set_tool_voltage(config->tool_voltage)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set tool voltage");
        return false;
    }

    // Step 2: Launch MoveIt
    RCLCPP_INFO(node_->get_logger(), "Launching MoveIt for %s gripper", gripper.c_str());
    launch_moveit_process(config->moveit_package,
                         "robot_bringup.launch.py",
                         robot_ip);

    // Step 3: Wait for MoveIt to be ready
    auto plan_client = node_->create_client<moveit_msgs::srv::GetMotionPlan>("/plan_kinematic_path");
    if (!plan_client->wait_for_service(30s)) {
        RCLCPP_ERROR(node_->get_logger(), "MoveIt planning service not ready within 30s");
        kill_current_process();
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "MoveIt ready");

    // Step 4: Load collision obstacles (REQUIRED for safety)
    std::string config_file = node_->get_parameter("obstacle_config_path").as_string();
    if (!config_file.empty() && config_file[0] != '/') {
        try {
            config_file = ament_index_cpp::get_package_share_directory("mtc_pipeline") + "/" + config_file;
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to resolve obstacle config path");
            kill_current_process();
            return false;
        }
    }
    if (config_file.empty() || !mtc_pipeline::loadPlanningSceneObstacles(node_->get_logger(), config_file)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load obstacles - aborting for safety");
        kill_current_process();
        return false;
    }

    // Step 5: Restart UR external_control program (voltage command stops it)
    if (!tool_interface_->restart_external_control()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to restart external_control");
        kill_current_process();
        return false;
    }

    current_gripper_ = gripper;
    RCLCPP_INFO(node_->get_logger(), "Robot ready with %s configuration", gripper.c_str());
    return true;
}

void MoveItLifecycleManager::kill_current_process()
{
    // (EXACT copy from orchestrator lines 411-433: kill_moveit_process)

    if (moveit_pid_ <= 0) return;

    // Send SIGTERM to process group
    kill(-moveit_pid_, SIGTERM);

    // Wait up to 2s for graceful exit
    auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline && kill(moveit_pid_, 0) == 0) {
        std::this_thread::sleep_for(50ms);
    }

    // Force kill if still alive
    if (kill(moveit_pid_, 0) == 0) {
        kill(-moveit_pid_, SIGKILL);
    }

    // Wait for process to fully exit
    int status;
    waitpid(moveit_pid_, &status, 0);
    moveit_pid_ = 0;
}

// ============================================================================
// PRIVATE HELPERS
// ============================================================================

pid_t MoveItLifecycleManager::launch_moveit_process(
    const std::string& package,
    const std::string& launch_file,
    const std::string& robot_ip)
{
    // (EXACT copy from orchestrator lines 375-409: launch_moveit_process)

    pid_t pid = fork();

    if (pid == 0) {
        // Child: create new process group and exec
        setsid();

        // Build argument array for direct execution (no shell!)
        std::string robot_ip_arg = "robot_ip:=" + robot_ip;
        char* args[] = {
            (char*)"ros2",
            (char*)"launch",
            (char*)package.c_str(),
            (char*)launch_file.c_str(),
            (char*)robot_ip_arg.c_str(),
            nullptr  // Must be null-terminated
        };

        // Execute directly without shell - no command injection possible
        execvp("ros2", args);

        // Only reached if exec fails
        _exit(1);
    }

    if (pid > 0) {
        moveit_pid_ = pid;
    }

    return pid;
}

}  // namespace core
}  // namespace mtc_pipeline
