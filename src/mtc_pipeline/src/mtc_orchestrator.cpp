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
    // Wait for ROS2 service to become available
    bool wait_for_service(rclcpp::Node::SharedPtr node, const std::string& service_name,std::chrono::seconds timeout = 30s) {

        auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
        return client->wait_for_service(timeout);
    }

    // Execute shell command and check if output contains expected string
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

    // Wait for MoveIt stack to be ready
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

    // Copy robot description parameters for orchestrator
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

    // Send play command to robot dashboard
    bool play_dashboard_client(rclcpp::Node::SharedPtr node) {
        RCLCPP_INFO(node->get_logger(), "Waiting for dashboard service...");
        if (!wait_for_service(node, "/dashboard_client/play", 30s)) {
            RCLCPP_ERROR(node->get_logger(), "Dashboard 'play' service not available!");
            return false;
        }
        
        auto client = node->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
        auto fut = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        
        if (rclcpp::spin_until_future_complete(node, fut, 5s) == rclcpp::FutureReturnCode::SUCCESS && fut.get()->success) {
            RCLCPP_INFO(node->get_logger(), "Dashboard 'play' called successfully.");
            return true;
        }
        
        RCLCPP_WARN(node->get_logger(), "Dashboard 'play' failed");
        return false;
    }

    // Get launch command for gripper type
    std::string launch_cmd_for_gripper(const std::string& g, const std::string& ip) {
        if (g == "none") return "ros2 launch ur_standalone_moveit_config move_group.launch.py robot_ip:=" + ip;
        if (g == "epick") return "ros2 launch ur_epick_moveit_config move_group.launch.py robot_ip:=" + ip;
        if (g == "hande") return "ros2 launch ur_hande_moveit_config move_group.launch.py robot_ip:=" + ip;
        return "";
    }
}

// Manages MoveIt configuration processes
class Orchestrator {
    std::vector<pid_t> active_pids_; 
    std::string current_gripper_ = "none"; 
    
public:
    // Launch new MoveIt configuration process
    pid_t launch(const std::string& cmd) {
        pid_t pid = fork(); 
        if (pid == 0) {
            execl("/usr/bin/setsid", "setsid", "bash", "-c", cmd.c_str(), (char*)nullptr);
            exit(1); 
        }
        active_pids_.push_back(pid);
        return pid;
    }

    // Gracefully terminate all processes
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
        
        for (pid_t pid : active_pids_) {
            waitpid(pid, nullptr, WNOHANG);
        }
        active_pids_.clear();
    }

    void set_current_gripper(const std::string& g) { current_gripper_ = g; }
    const std::string& get_current_gripper() const { return current_gripper_; }
};

Orchestrator* global_orch = nullptr;

// Handle Ctrl+C 
void sigint_handler(int) {
    std::cerr << "\n[Orchestrator] SIGINT received. Shutting down...\n";
    if (global_orch) global_orch->kill_all_and_wait();
    std::exit(0);
}

// Switch to different gripper configuration
bool switch_gripper(Orchestrator& orch, const std::string& new_gripper, const std::string& robot_ip, 
                   rclcpp::Node::SharedPtr node) {
    if (orch.get_current_gripper() == new_gripper) return true;
    
    orch.kill_all_and_wait();
    orch.launch(launch_cmd_for_gripper(new_gripper, robot_ip));
    
    if (!wait_for_moveit_ready(node, 30s) ||
        !update_robot_description_from("move_group", node))
        return false;
    
    play_dashboard_client(node); 
    orch.set_current_gripper(new_gripper);
    return true;
}

// Execute a single task step
bool execute_step(const std::string& action, const nlohmann::json& step, const nlohmann::json& poses, 
                  rclcpp::Node::SharedPtr node, Orchestrator& orch, const std::string& robot_ip,
                  PickPlaceStages& pick_place, ToolExchangeStages& tool_exch, 
                  MoveToStages& moveto, EndEffectorStages& end_effector) {
    
    // Handle tool exchange tasks
    if (action == "tool_exchange") {
        const std::string operation = step.value("operation", "");
        const std::string requested_tool = step.value("gripper", orch.get_current_gripper());

        if (!update_robot_description_from("move_group", node) || !tool_exch.run(step, poses, node))
            return false;

        if (operation == "dock") {
            return switch_gripper(orch, "none", robot_ip, node);
        } else if (operation == "load") {
            return switch_gripper(orch, requested_tool, robot_ip, node);
        }
        return true;
    }

    // Handle pick and place tasks
    if (action == "pick_and_place") {
        std::string need = step.value("gripper", orch.get_current_gripper());
        if (!switch_gripper(orch, need, robot_ip, node))
            return false;

        return update_robot_description_from("move_group", node) && pick_place.run(step, poses, node);
    }

    // Handle simple move-to tasks
    if (action == "moveto") {
        return update_robot_description_from("move_group", node) && moveto.run(step, poses, node);
    }

    // Handle end effector operations
    if (action == "end_effector") {
        return end_effector.run(step, poses, node);
    }

    return false;
}

int main(int argc, char** argv) {
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mtc_orchestrator", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Read configuration file
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

    // Setup orchestrator
    Orchestrator orch;
    global_orch = &orch;
    signal(SIGINT, sigint_handler); 

    // Start MoveIt configuration
    orch.kill_all_and_wait();
    orch.launch(launch_cmd_for_gripper(start_gripper, robot_ip));
    
    if (!wait_for_moveit_ready(node, 30s)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize MoveIt stack!");
        return 1;
    }
    
    // Activate robot controller
    RCLCPP_INFO(node->get_logger(), "Activating scaled_joint_trajectory_controller...");
    system("ros2 control switch_controllers --activate scaled_joint_trajectory_controller");
    
    play_dashboard_client(node);
    
    if (!update_robot_description_from("move_group", node)) return 1;

    orch.set_current_gripper(start_gripper);

    // Create task handlers
    PickPlaceStages pick_place(node, cfg);
    ToolExchangeStages tool_exch(node, cfg);
    MoveToStages moveto(node, cfg);
    EndEffectorStages end_effector(node, cfg);

    // Execute task sequence
    for (const auto& step : sequence) {
        const std::string action = step.at("action");
        
        if (!execute_step(action, step, poses, node, orch, robot_ip, pick_place, tool_exch, moveto, end_effector)) {
            RCLCPP_ERROR(node->get_logger(), "%s step failed – aborting.", action.c_str());
            orch.kill_all_and_wait();
            rclcpp::shutdown();
            return 1;
        }
    }

    orch.kill_all_and_wait();
    rclcpp::shutdown();
    return 0;
}
