#include "mtc_pipeline/pick_place_stages.hpp"
#include "mtc_pipeline/tool_exchange_stages.hpp"
#include "mtc_pipeline/moveto_stages.hpp"
#include "mtc_pipeline/end_effector_stages.hpp"
#include <fstream>          
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <vector>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

namespace {
    bool wait_for_service(rclcpp::Node::SharedPtr node, const std::string& service_name,std::chrono::seconds timeout = 30s) {

        auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
        return client->wait_for_service(timeout);
    }

    bool check_command_output(const std::string& cmd, const std::string& expected) {
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) return false; 
        char buf[128]; 
        bool found = false;
        while (fgets(buf, 128, pipe)) {
            if (std::string(buf).find(expected) != std::string::npos) {
                found = true;
                break;
            }
        }
        pclose(pipe);
        return found;
    }


    bool wait_for_moveit_ready(rclcpp::Node::SharedPtr node, std::chrono::seconds timeout = 30s) {
        RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt to become ready...");
        
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < timeout) {

            if (!check_command_output("ros2 node list", "move_group") ||
                !check_command_output("ros2 topic list | grep joint_states", "joint_states")) {
                std::this_thread::sleep_for(100ms);
                continue; 
            }
            
            std::this_thread::sleep_for(5s);
            
            RCLCPP_INFO(node->get_logger(), "MoveIt is ready!");
            return true;
        }
        
        RCLCPP_ERROR(node->get_logger(), "MoveIt failed to become ready!");
        return false;
    }

    // Sync robot description 
    bool update_robot_description_from(const std::string& source_node, rclcpp::Node::SharedPtr node) {
        auto client = std::make_shared<rclcpp::SyncParametersClient>(node, source_node);
        if (!client->wait_for_service(5s)) {
            RCLCPP_ERROR(node->get_logger(), "Could not contact parameter service of %s", source_node.c_str());
            return false;
        }

        try {
            auto urdf = client->get_parameter<std::string>("robot_description");
            auto srdf = client->get_parameter<std::string>("robot_description_semantic");
            node->set_parameters({{"robot_description", urdf}, {"robot_description_semantic", srdf}});
            RCLCPP_INFO(node->get_logger(), "Robot params synced from [%s]", source_node.c_str());
            return true;
        } catch (...) {
            RCLCPP_ERROR(node->get_logger(), "Failed to get robot description from %s", source_node.c_str());
            return false;
        }
    }

    // Send a "play" command to the robot's dashboard
    bool play_dashboard_client(rclcpp::Node::SharedPtr node) {
        RCLCPP_INFO(node->get_logger(), "Waiting for dashboard service...");
        if (!wait_for_service(node, "/dashboard_client/play", 30s)) {
            RCLCPP_ERROR(node->get_logger(), "Dashboard 'play' service not available!");
            return false;
        }
        
        // Create a client and send the play request
        auto client = node->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
        auto fut = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        
        // Wait for the response
        if (rclcpp::spin_until_future_complete(node, fut, 5s) == rclcpp::FutureReturnCode::SUCCESS && fut.get()->success) {
            RCLCPP_INFO(node->get_logger(), "Dashboard 'play' called successfully.");
            return true;
        }
        
        RCLCPP_WARN(node->get_logger(), "Dashboard 'play' failed, but continuing...");
        return false;
    }

    std::string launch_cmd_for_gripper(const std::string& g, const std::string& ip) {
        if (g == "none") return "ros2 launch ur_standalone_moveit_config move_group.launch.py robot_ip:=" + ip;
        if (g == "epick") return "ros2 launch ur_epick_moveit_config move_group.launch.py robot_ip:=" + ip;
        if (g == "hande") return "ros2 launch ur_hande_moveit_config move_group.launch.py robot_ip:=" + ip;
        return "";
    }
}


class Orchestrator {
    std::vector<pid_t> active_pids_; 
    std::string current_gripper_ = "none"; 
    
public:
    pid_t launch(const std::string& cmd) {
        pid_t pid = fork(); 
        if (pid == 0) {
            execl("/usr/bin/setsid", "setsid", "bash", "-c", cmd.c_str(), (char*)nullptr); // launch required gripper config
            exit(1); 
        }
        active_pids_.push_back(pid);
        return pid;
    }

    void kill_all_and_wait()
    {
        for (pid_t pid : active_pids_)
            ::kill(-pid, SIGINT);
        
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(15);
        
        while (std::chrono::steady_clock::now() - start_time < timeout) {
            bool all_terminated = true;
            for (pid_t pid : active_pids_) {
                int status;
                if (waitpid(pid, &status, WNOHANG) == 0) {
                    all_terminated = false;
                    break;
                }
            }
            if (all_terminated) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
       
        // Force kill any remaining processes
        for (pid_t pid : active_pids_) {
            int status;
            if (waitpid(pid, &status, WNOHANG) == 0) {
                ::kill(-pid, SIGKILL);
                waitpid(pid, nullptr, 0);
            } else {
                waitpid(pid, nullptr, 0);
            }
        }
        active_pids_.clear();
    }

    // Wait for a specific ROS2 node to appear in the system
    bool wait_for_node(const std::string& node_name, int max_tries = 20, int interval_sec = 1) {
        for (int i = 0; i < max_tries; ++i) {
            if (check_command_output("ros2 node list", node_name)) return true;
            sleep(interval_sec);
        }
        return false;
    }

    // Getter and setter for current gripper
    void set_current_gripper(const std::string& g) { current_gripper_ = g; }
    const std::string& get_current_gripper() const { return current_gripper_; }
};

// Global orchestrator for signal handling
Orchestrator* global_orch = nullptr;
void sigint_handler(int) {
    std::cerr << "\n[Orchestrator] SIGINT received. Shutting down...\n";
    if (global_orch) global_orch->kill_all_and_wait();
    std::exit(0);
}

// Helper function to switch between different gripper configurations
// This involves stopping the current MoveIt stack and starting a new one
bool switch_gripper(Orchestrator& orch, const std::string& new_gripper, const std::string& robot_ip, 
                   rclcpp::Node::SharedPtr node) {
    // If we're already using the requested gripper, do nothing
    if (orch.get_current_gripper() == new_gripper) return true;
    
    // Stop all current processes
    orch.kill_all_and_wait();
    // Start the new MoveIt configuration
    orch.launch(launch_cmd_for_gripper(new_gripper, robot_ip));
    
    // Wait for everything to be ready
    if (!orch.wait_for_node("move_group") || !wait_for_moveit_ready(node, 30s) ||
        !update_robot_description_from("move_group", node))
        return false;
    
    // Tell the robot to start accepting commands
    play_dashboard_client(node); // Ignore failure, continue anyway
    orch.set_current_gripper(new_gripper);
    return true;
}

// Execute a single task step from the sequence
// This is the main function that handles all different types of robot tasks
bool execute_step(const std::string& action, const nlohmann::json& step, const nlohmann::json& poses, 
                  rclcpp::Node::SharedPtr node, Orchestrator& orch, const std::string& robot_ip,
                  PickPlaceStages& pick_place, ToolExchangeStages& tool_exch, 
                  MoveToStages& moveto, EndEffectorStages& end_effector) {
    
    // Handle tool exchange tasks (attaching/detaching grippers)
    if (action == "tool_exchange") {
        const std::string operation = step.value("operation", "");
        const std::string requested_tool = step.value("gripper", orch.get_current_gripper());

        // First, plan and execute the physical tool exchange motion
        if (!update_robot_description_from("move_group", node) || !tool_exch.run(step, poses, node))
            return false;

        // Then switch the software configuration if needed
        if (operation == "dock") {
            // Dock operation means removing the tool (going to "none" gripper)
            return switch_gripper(orch, "none", robot_ip, node);
        } else if (operation == "load") {
            // Load operation means attaching a new tool
            return switch_gripper(orch, requested_tool, robot_ip, node);
        }
        return true;
    }

    // Handle pick and place tasks
    if (action == "pick_and_place") {
        std::string need = step.value("gripper", orch.get_current_gripper());
        // Switch to the required gripper if needed
        if (!switch_gripper(orch, need, robot_ip, node))
            return false;

        // Execute the pick and place motion
        return update_robot_description_from("move_group", node) && pick_place.run(step, poses, node);
    }

    // Handle simple move-to tasks
    if (action == "moveto") {
        return update_robot_description_from("move_group", node) && moveto.run(step, poses, node);
    }

    // Handle end effector operations (like opening/closing gripper)
    if (action == "end_effector") {
        return end_effector.run(step, poses, node);
    }

    return false; // Unknown action type
}

// Main function - this is where the program starts
int main(int argc, char** argv) {
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mtc_orchestrator", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Read from JSON file
    std::string cfg_file = "./script.json";
    node->get_parameter("poses_file", cfg_file);
    std::ifstream ifs(cfg_file);
    if (!ifs) { 
        RCLCPP_ERROR(node->get_logger(), "Cannot open %s", cfg_file.c_str()); 
        return 1; 
    }

    nlohmann::json cfg; 
    ifs >> cfg;
    const auto& poses = cfg.at("poses");
    const auto& sequence = cfg.at("sequence");

    std::string robot_ip = "192.168.1.101";
    node->get_parameter("robot_ip", robot_ip);
    std::string start_gripper = cfg.value("start_gripper", "none");


    Orchestrator orch;
    global_orch = &orch;
    signal(SIGINT, sigint_handler); 

    // Start the initial MoveIt configuration
    orch.kill_all_and_wait(); // Clean up any existing processes
    orch.launch(launch_cmd_for_gripper(start_gripper, robot_ip)); // Start MoveIt
    
    // Wait for everything to be ready
    if (!orch.wait_for_node("move_group") || !wait_for_moveit_ready(node, 30s)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize MoveIt stack!");
        return 1;
    }
    
    // Activate the robot controller
    RCLCPP_INFO(node->get_logger(), "Activating scaled_joint_trajectory_controller...");
    system("ros2 control switch_controllers --activate scaled_joint_trajectory_controller");
    
    // Tell the robot to start accepting commands
    play_dashboard_client(node); // Ignore failure, continue anyway
    
    // Get the robot description from MoveIt
    if (!update_robot_description_from("move_group", node)) return 1;

    orch.set_current_gripper(start_gripper);

    // Create the task stage objects that handle different types of robot movements
    PickPlaceStages pick_place(node, cfg);
    ToolExchangeStages tool_exch(node, cfg);
    MoveToStages moveto(node, cfg);
    EndEffectorStages end_effector(node, cfg);

    // Execute each task in the sequence
    for (const auto& step : sequence) {
        const std::string action = step.at("action");
        
        // Try to execute this step
        if (!execute_step(action, step, poses, node, orch, robot_ip, pick_place, tool_exch, moveto, end_effector)) {
            RCLCPP_ERROR(node->get_logger(), "%s step failed – aborting.", action.c_str());
            orch.kill_all_and_wait(); // Clean up
            rclcpp::shutdown();
            return 1;
        }
    }

    // All tasks completed successfully
    orch.kill_all_and_wait(); // Clean up
    rclcpp::shutdown();
    return 0;
}
