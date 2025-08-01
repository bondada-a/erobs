#include "mtc_pipeline/pick_place_stages.hpp"
#include "mtc_pipeline/tool_exchange_stages.hpp"
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

// --- Helper to declare parameter if needed ---
void declare_if_needed(rclcpp::Node::SharedPtr node, const std::string& name) {
    if (!node->has_parameter(name)) {
        node->declare_parameter(name, std::string{""});
    }
}

// --- Helper to sync robot/planning params from running MoveIt stack ---
bool update_robot_description_from(const std::string& source_node, rclcpp::Node::SharedPtr node)
{
    using namespace std::chrono_literals;
    auto client = std::make_shared<rclcpp::SyncParametersClient>(node, source_node);
    if (!client->wait_for_service(5s)) {
        RCLCPP_ERROR(node->get_logger(), "Could not contact parameter service of %s", source_node.c_str());
        return false;
    }
    auto urdf  = client->get_parameter<std::string>("robot_description");
    auto srdf  = client->get_parameter<std::string>("robot_description_semantic");
    auto kin   = client->has_parameter("robot_description_kinematics") ?
                 client->get_parameter<std::string>("robot_description_kinematics") : "";
    auto jlim  = client->has_parameter("robot_description_planning_joint_limits") ?
                 client->get_parameter<std::string>("robot_description_planning_joint_limits") : "";
    auto pipe  = client->has_parameter("moveit_cpp.planning_pipelines") ?
                 client->get_parameter<std::string>("moveit_cpp.planning_pipelines") : "";

    // Declare parameters before setting them!
    declare_if_needed(node, "robot_description");
    declare_if_needed(node, "robot_description_semantic");
    if (!kin.empty()) declare_if_needed(node, "robot_description_kinematics");
    if (!jlim.empty()) declare_if_needed(node, "robot_description_planning_joint_limits");
    if (!pipe.empty()) declare_if_needed(node, "moveit_cpp.planning_pipelines");

    std::vector<rclcpp::Parameter> params = {
        rclcpp::Parameter("robot_description", urdf),
        rclcpp::Parameter("robot_description_semantic", srdf)
    };
    if (!kin.empty()) params.emplace_back("robot_description_kinematics", kin);
    if (!jlim.empty()) params.emplace_back("robot_description_planning_joint_limits", jlim);
    if (!pipe.empty()) params.emplace_back("moveit_cpp.planning_pipelines", pipe);
    node->set_parameters(params);
    RCLCPP_INFO(node->get_logger(), "Synchronized robot model and MoveIt planning config from [%s]", source_node.c_str());
    return true;
}

// --- Helper function to call dashboard /play ---
bool play_dashboard_client(rclcpp::Node::SharedPtr node) {
    auto client = node->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
    if (!client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "Dashboard 'play' service not available!");
        return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call dashboard 'play' service");
        return false;
    }
    auto response = result.get();
    if (!response->success) {
        RCLCPP_ERROR(node->get_logger(), "Dashboard 'play' service responded with failure: %s", response->message.c_str());
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "Dashboard 'play' service called successfully.");
    return true;
}

class Orchestrator {
    std::vector<pid_t> active_pids_;
    std::string current_gripper_ = "none";
public:
    pid_t launch(const std::string& cmd) {
        pid_t pid = fork();
        if (pid == 0) {
            execl("/usr/bin/setsid", "setsid", "bash", "-c", cmd.c_str(), (char*)nullptr);
            exit(1);
        }
        active_pids_.push_back(pid);
        return pid;
    }

    void kill_all_and_wait() {
        for (pid_t pid : active_pids_) {
            ::kill(-pid, SIGINT);
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
        for (pid_t pid : active_pids_) {
            int status;
            pid_t result = waitpid(pid, &status, WNOHANG);
            if (result == 0) {
                ::kill(-pid, SIGKILL);
                waitpid(pid, nullptr, 0);
            } else {
                waitpid(pid, nullptr, 0);
            }
        }
        active_pids_.clear();
    }

    bool wait_for_node(const std::string& node_name, int max_tries = 20, int interval_sec = 1) {
        for (int i = 0; i < max_tries; ++i) {
            FILE* pipe = popen("ros2 node list", "r");
            if (!pipe) return false;
            char buffer[128];
            bool found = false;
            while (fgets(buffer, 128, pipe) != nullptr) {
                std::string line(buffer);
                if (line.find(node_name) != std::string::npos) {
                    found = true;
                    break;
                }
            }
            pclose(pipe);
            if (found) return true;
            sleep(interval_sec);
        }
        return false;
    }

    void set_current_gripper(const std::string& gripper) {
        current_gripper_ = gripper;
    }

    std::string get_current_gripper() const {
        return current_gripper_;
    }
};

// GLOBAL for signal handling
Orchestrator* global_orch = nullptr;
void sigint_handler(int) {
    std::cerr << "\n[Orchestrator] SIGINT received. Shutting down...\n";
    if (global_orch) global_orch->kill_all_and_wait();
    std::exit(0);
}

std::string launch_cmd_for_gripper(const std::string& gripper) {
    if (gripper == "none") {
        return "ros2 launch ur_standalone_moveit_config move_group.launch.py robot_ip:=192.168.56.101";
    } else if (gripper == "epick") {
        return "ros2 launch ur_epick_moveit_config move_group.launch.py";
    } else if (gripper == "hande") {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        return "ros2 launch ur_hande_moveit_config move_group.launch.py robot_ip:=192.168.56.101";
    }
    return "";
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("mtc_orchestrator", options);

    Orchestrator orch;
    global_orch = &orch;
    signal(SIGINT, sigint_handler);

    // Load JSON file
    std::string config_file = "./script.json";
    node->get_parameter("poses_file", config_file);

    std::ifstream ifs(config_file);
    if (!ifs) {
        RCLCPP_ERROR(node->get_logger(), "Unable to open config file: %s", config_file.c_str());
        return 1;
    }
    nlohmann::json config;
    ifs >> config;

    if (!config.contains("poses") || !config.contains("sequence")) {
        RCLCPP_ERROR(node->get_logger(), "JSON must contain 'poses' and 'sequence'");
        return 1;
    }

    std::string start_gripper = config.value("start_gripper", "none");
    const nlohmann::json& poses = config["poses"];
    const nlohmann::json& sequence = config["sequence"];

    // 1. Start with no gripper
    orch.kill_all_and_wait();
    std::string initial_cmd = launch_cmd_for_gripper("none");
    if (!initial_cmd.empty()) {
        orch.launch(initial_cmd);
        orch.wait_for_node("move_group");

        // Play dashboard after launching stack
        if (!play_dashboard_client(node)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to play dashboard client after initial stack launch");
            return 1;
        }

        // ---- DYNAMICALLY sync robot/planning params from the stack! ----
        if (!update_robot_description_from("move_group", node)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to update robot_description from move_group");
            return 1;
        }
        orch.set_current_gripper("none");
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }

    // Only now that params are synced, construct your stages
    PickPlaceStages pick_place_module(node, config);
    ToolExchangeStages tool_exchange_module(node, config);

    for (const auto& step : sequence) {
        std::string action = step.at("action").get<std::string>();
        bool success = false;
        const int max_attempts = 1;
        int attempt = 0;

        for (; attempt < max_attempts; ++attempt) {
            if (action == "tool_exchange") {
                std::string new_gripper = step.value("gripper", "none");
                if (orch.get_current_gripper() != "none") {
                    orch.kill_all_and_wait();
                    std::string cmd = launch_cmd_for_gripper("none");
                    orch.launch(cmd);
                    orch.wait_for_node("move_group");

                    // Play dashboard after launching stack (toolless)
                    if (!play_dashboard_client(node)) {
                        RCLCPP_ERROR(node->get_logger(), "Failed to play dashboard client after launching stack (toolless)");
                        break;
                    }

                    if (!update_robot_description_from("move_group", node)) {
                        RCLCPP_ERROR(node->get_logger(), "Failed to update robot_description for toolless state");
                        break;
                    }
                    orch.set_current_gripper("none");
                }

                // sync params before tool exchange planning!
                if (!update_robot_description_from("move_group", node)) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to update robot_description before tool exchange");
                    break;
                }
                success = tool_exchange_module.run(step, poses, node);
                if (!success) break;

                // Launch new gripper stack
                orch.kill_all_and_wait();
                std::string gripper_cmd = launch_cmd_for_gripper(new_gripper);
                if (gripper_cmd.empty()) {
                    RCLCPP_ERROR(node->get_logger(), "Unknown gripper: %s", new_gripper.c_str());
                    break;
                }
                orch.launch(gripper_cmd);
                orch.wait_for_node("move_group");

                // Play dashboard after launching stack (tool attach)
                if (!play_dashboard_client(node)) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to play dashboard client after launching stack (tool attach)");
                    break;
                }

                if (!update_robot_description_from("move_group", node)) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to update robot_description for %s", new_gripper.c_str());
                    break;
                }
                orch.set_current_gripper(new_gripper);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                success = true;
            } else if (action == "pick_and_place") {
                std::string required_gripper = step.value("gripper", orch.get_current_gripper());
                if (required_gripper != orch.get_current_gripper()) {
                    orch.kill_all_and_wait();
                    std::this_thread::sleep_for(std::chrono::seconds(20));
                    std::string gripper_cmd = launch_cmd_for_gripper(required_gripper);
                    if (gripper_cmd.empty()) {
                        RCLCPP_ERROR(node->get_logger(), "Unknown gripper: %s", required_gripper.c_str());
                        break;
                    }
                    orch.launch(gripper_cmd);
                    orch.wait_for_node("move_group");

                    // Play dashboard after launching stack (pick_and_place)
                    if (!play_dashboard_client(node)) {
                        RCLCPP_ERROR(node->get_logger(), "Failed to play dashboard client after launching stack (pick_and_place)");
                        break;
                    }

                    if (!update_robot_description_from("move_group", node)) {
                        RCLCPP_ERROR(node->get_logger(), "Failed to update robot_description for %s", required_gripper.c_str());
                        break;
                    }
                    orch.set_current_gripper(required_gripper);
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                }
                // Always sync params before planning!
                if (!update_robot_description_from("move_group", node)) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to update robot_description before pick_and_place");
                    break;
                }
                std::this_thread::sleep_for(std::chrono::seconds(25));
                success = pick_place_module.run(step, poses, node);
            }
            if (success) break;
            RCLCPP_WARN(node->get_logger(),
                "Step %s failed on attempt %d/%d, retrying...", action.c_str(), attempt + 1, max_attempts);
        }

        if (!success) {
            RCLCPP_ERROR(node->get_logger(),
                "%s step failed after %d attempts", action.c_str(), max_attempts);
            orch.kill_all_and_wait();
            rclcpp::shutdown();
            return 1;
        }
    }

    orch.kill_all_and_wait();
    rclcpp::shutdown();
    return 0;
}
