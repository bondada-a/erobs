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
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Declares a parameter if it doesn't exist
void declare_if_needed(rclcpp::Node::SharedPtr node, const std::string& name)
{
    if (!node->has_parameter(name))
        node->declare_parameter(name, std::string{""});
}

// Wait for a service to become available with timeout
bool wait_for_service(rclcpp::Node::SharedPtr node, const std::string& service_name, 
                     std::chrono::seconds timeout = std::chrono::seconds(30))
{
    auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
    return client->wait_for_service(timeout);
}

// Ensure scaled_joint_trajectory_controller is active
bool ensure_controller_active(rclcpp::Node::SharedPtr node, 
                             std::chrono::seconds timeout = std::chrono::seconds(30))
{
    using namespace std::chrono_literals;
    
    RCLCPP_INFO(node->get_logger(), "Ensuring scaled_joint_trajectory_controller is active...");
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        // Check current controller state
        FILE* pipe = popen("ros2 control list_controllers | grep scaled_joint_trajectory_controller", "r");
        if (!pipe) {
            std::this_thread::sleep_for(500ms);
            continue;
        }
        
        char buf[128];
        std::string controller_state;
        while (fgets(buf, 128, pipe)) {
            std::string line(buf);
            if (line.find("scaled_joint_trajectory_controller") != std::string::npos) {
                controller_state = line;
                break;
            }
        }
        pclose(pipe);
        
        if (controller_state.find("active") != std::string::npos) {
            RCLCPP_INFO(node->get_logger(), "scaled_joint_trajectory_controller is already active!");
            return true;
        }
        
        if (controller_state.find("inactive") != std::string::npos) {
            RCLCPP_INFO(node->get_logger(), "Attempting to activate scaled_joint_trajectory_controller...");
            
            // Try to activate the controller
            std::string activate_cmd = "ros2 control switch_controllers --activate scaled_joint_trajectory_controller";
            int result = system(activate_cmd.c_str());
            
            if (result == 0) {
                RCLCPP_INFO(node->get_logger(), "Controller activation command sent successfully");
                // Wait a bit for the activation to take effect
                std::this_thread::sleep_for(1s);
            } else {
                RCLCPP_WARN(node->get_logger(), "Failed to activate controller, retrying...");
            }
        }
        
        std::this_thread::sleep_for(500ms);
    }
    
    RCLCPP_ERROR(node->get_logger(), "Failed to ensure controller is active within timeout!");
    return false;
}

// Wait for robot to reach a stable state (joint velocities near zero)
bool wait_for_robot_stable(rclcpp::Node::SharedPtr node, 
                          std::chrono::seconds timeout = std::chrono::seconds(30),
                          double velocity_threshold = 0.01)
{
    using namespace std::chrono_literals;
    
    std::promise<bool> promise;
    auto future = promise.get_future();
    
    auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [&promise, velocity_threshold](const sensor_msgs::msg::JointState::SharedPtr msg) {
            bool stable = true;
            for (const auto& velocity : msg->velocity) {
                if (std::abs(velocity) > velocity_threshold) {
                    stable = false;
                    break;
                }
            }
            if (stable) {
                promise.set_value(true);
            }
        });
    
    // Spin the node to process messages
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        rclcpp::spin_some(node);
        if (future.wait_for(100ms) == std::future_status::ready) {
            return true;
        }
    }
    
    return false;
}

// Wait for MoveIt to be ready (multiple conditions)
bool wait_for_moveit_ready(rclcpp::Node::SharedPtr node, 
                          std::chrono::seconds timeout = std::chrono::seconds(30))
{
    using namespace std::chrono_literals;
    
    RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt to become ready...");
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        // Check if move_group node is running
        FILE* pipe = popen("ros2 node list", "r");
        if (!pipe) continue;
        
        char buf[128];
        bool move_group_found = false;
        while (fgets(buf, 128, pipe)) {
            if (std::string(buf).find("move_group") != std::string::npos) {
                move_group_found = true;
                break;
            }
        }
        pclose(pipe);
        
        if (!move_group_found) {
            std::this_thread::sleep_for(100ms);
            continue;
        }
        
        // Check if joint_states topic is publishing
        pipe = popen("ros2 topic list | grep joint_states", "r");
        if (!pipe) continue;
        
        bool joint_states_found = false;
        while (fgets(buf, 128, pipe)) {
            if (std::string(buf).find("joint_states") != std::string::npos) {
                joint_states_found = true;
                break;
            }
        }
        pclose(pipe);
        
        if (!joint_states_found) {
            std::this_thread::sleep_for(100ms);
            continue;
        }
        
        // Check if planning scene topic is publishing
        pipe = popen("ros2 topic list | grep planning_scene", "r");
        if (!pipe) continue;
        
        bool planning_scene_found = false;
        while (fgets(buf, 128, pipe)) {
            if (std::string(buf).find("planning_scene") != std::string::npos) {
                planning_scene_found = true;
                break;
            }
        }
        pclose(pipe);
        
        if (!planning_scene_found) {
            std::this_thread::sleep_for(100ms);
            continue;
        }
        
        // NEW: Check if scaled_joint_trajectory_controller is running
        pipe = popen("ros2 control list_controllers | grep scaled_joint_trajectory_controller", "r");
        if (!pipe) continue;
        
        bool controller_running = false;
        while (fgets(buf, 128, pipe)) {
            std::string line(buf);
            if (line.find("scaled_joint_trajectory_controller") != std::string::npos && 
                line.find("active") != std::string::npos) {
                controller_running = true;
                break;
            }
        }
        pclose(pipe);
        
        if (!controller_running) {
            RCLCPP_INFO(node->get_logger(), "Waiting for scaled_joint_trajectory_controller to be active...");
            std::this_thread::sleep_for(100ms);
            continue;
        }
        
        // Try to get a message from joint_states to confirm it's actually publishing
        std::promise<bool> joint_states_promise;
        auto joint_states_future = joint_states_promise.get_future();
        
        auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [&joint_states_promise](const sensor_msgs::msg::JointState::SharedPtr) {
                joint_states_promise.set_value(true);
            });
        
        // Wait for joint states message with short timeout
        auto check_start = std::chrono::steady_clock::now();
        bool got_joint_states = false;
        while (std::chrono::steady_clock::now() - check_start < 2s) {
            rclcpp::spin_some(node);
            if (joint_states_future.wait_for(100ms) == std::future_status::ready) {
                got_joint_states = true;
                break;
            }
        }
        
        if (got_joint_states) {
            RCLCPP_INFO(node->get_logger(), "MoveIt is ready!");
            return true;
        }
        
        std::this_thread::sleep_for(100ms);
    }
    
    RCLCPP_ERROR(node->get_logger(), "MoveIt failed to become ready within timeout!");
    return false;
}

bool update_robot_description_from(const std::string& source_node,
                                   rclcpp::Node::SharedPtr node)
{
    using namespace std::chrono_literals;
    auto client = std::make_shared<rclcpp::SyncParametersClient>(node, source_node);
    if (!client->wait_for_service(5s)) {
        RCLCPP_ERROR(node->get_logger(),
                     "Could not contact parameter service of %s",
                     source_node.c_str());
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
    auto plugin = client->has_parameter("planning_plugin") ?
                  client->get_parameter<std::string>("planning_plugin") : "";
    auto ompl_plugin = client->has_parameter("ompl.planning_plugin") ?
                       client->get_parameter<std::string>("ompl.planning_plugin") : "";

    declare_if_needed(node, "robot_description");
    declare_if_needed(node, "robot_description_semantic");
    if (!kin.empty())          declare_if_needed(node, "robot_description_kinematics");
    if (!jlim.empty())         declare_if_needed(node, "robot_description_planning_joint_limits");
    if (!pipe.empty())         declare_if_needed(node, "moveit_cpp.planning_pipelines");
    if (!plugin.empty())       declare_if_needed(node, "planning_plugin");
    if (!ompl_plugin.empty())  declare_if_needed(node, "ompl.planning_plugin");

    std::vector<rclcpp::Parameter> params{
        {"robot_description", urdf},
        {"robot_description_semantic", srdf}
    };
    if (!kin.empty())  params.emplace_back("robot_description_kinematics", kin);
    if (!jlim.empty()) params.emplace_back("robot_description_planning_joint_limits", jlim);
    if (!pipe.empty()) params.emplace_back("moveit_cpp.planning_pipelines", pipe);
    if (!plugin.empty()) params.emplace_back("planning_plugin", plugin);
    if (!ompl_plugin.empty()) params.emplace_back("ompl.planning_plugin", ompl_plugin);

    node->set_parameters(params);
    RCLCPP_INFO(node->get_logger(), "Robot/planning params synced from [%s].", source_node.c_str());
    return true;
}

bool play_dashboard_client(rclcpp::Node::SharedPtr node)
{
    // Wait for dashboard service to be available instead of hardcoded sleep
    RCLCPP_INFO(node->get_logger(), "Waiting for dashboard service...");
    if (!wait_for_service(node, "/dashboard_client/play", std::chrono::seconds(30))) {
        RCLCPP_ERROR(node->get_logger(), "Dashboard 'play' service not available!");
        return false;
    }
    
    // Try multiple times with increasing delays
    for (int attempt = 1; attempt <= 3; ++attempt) {
        RCLCPP_INFO(node->get_logger(), "Attempting dashboard 'play' (attempt %d/3)...", attempt);
        
        auto client = node->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
        auto fut = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        
        if (rclcpp::spin_until_future_complete(node, fut, std::chrono::seconds(5)) ==
            rclcpp::FutureReturnCode::SUCCESS && fut.get()->success) {
            RCLCPP_INFO(node->get_logger(), "Dashboard 'play' called successfully.");
            return true;
        } else {
            RCLCPP_WARN(node->get_logger(), "Dashboard 'play' attempt %d failed, retrying...", attempt);
            if (attempt < 3) {
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }
    }
    
    RCLCPP_ERROR(node->get_logger(), "Dashboard 'play' call failed after 3 attempts!");
    return false;
}

/* ============================================================
 *  Orchestrator class
 * ============================================================
 */
class Orchestrator
{
    std::vector<pid_t> active_pids_;
    std::string current_gripper_ = "none";
public:
    pid_t launch(const std::string& cmd)
    {
        pid_t pid = fork();
        if (pid == 0) {
            execl("/usr/bin/setsid", "setsid", "bash", "-c", cmd.c_str(), (char*)nullptr);
            exit(1);
        }
        active_pids_.push_back(pid);
        return pid;
    }

    void kill_all_and_wait()
    {
        for (pid_t pid : active_pids_)
            ::kill(-pid, SIGINT);
        
        // Wait for processes to terminate gracefully
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(10);
        
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

    bool wait_for_node(const std::string& node_name,
                       int max_tries = 20, int interval_sec = 1)
    {
        for (int i = 0; i < max_tries; ++i) {
            FILE* pipe = popen("ros2 node list", "r");
            if (!pipe) return false;
            char buf[128];
            bool found = false;
            while (fgets(buf, 128, pipe))
                if (std::string(buf).find(node_name) != std::string::npos) { found = true; break; }
            pclose(pipe);
            if (found) return true;
            sleep(interval_sec);
        }
        return false;
    }

    void  set_current_gripper(const std::string& g) { current_gripper_ = g; }
    const std::string& get_current_gripper() const  { return current_gripper_; }
};

/* ============================================================
 *  globals
 * ============================================================
 */
Orchestrator* global_orch = nullptr;
void sigint_handler(int)
{
    std::cerr << "\n[Orchestrator] SIGINT received. Shutting down...\n";
    if (global_orch) global_orch->kill_all_and_wait();
    std::exit(0);
}

std::string launch_cmd_for_gripper(const std::string& g, const std::string& ip)
{
    if (g == "none")  return "ros2 launch ur_standalone_moveit_config move_group.launch.py robot_ip:=" + ip;
    if (g == "epick") return "ros2 launch ur_epick_moveit_config      move_group.launch.py robot_ip:=" + ip;
    if (g == "hande") return "ros2 launch ur_hande_moveit_config      move_group.launch.py robot_ip:=" + ip;
    return "";
}

/* ============================================================
 *  main
 * ============================================================
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mtc_orchestrator",
                                               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    /* ---------- read JSON script ---------- */
    std::string cfg_file = "./script.json";
    node->get_parameter("poses_file", cfg_file);
    std::ifstream ifs(cfg_file);
    if (!ifs) { 
        RCLCPP_ERROR(node->get_logger(), "Cannot open %s", cfg_file.c_str()); 
        return 1; 
    }

    nlohmann::json cfg; 
    ifs >> cfg;
    const auto& poses    = cfg.at("poses");
    const auto& sequence = cfg.at("sequence");

    /* ---------- parameters ---------- */
    std::string robot_ip = "192.168.1.101";
    node->get_parameter("robot_ip", robot_ip);

    std::string start_gripper = cfg.value("start_gripper", "none");

    /* ---------- setup orchestrator ---------- */
    Orchestrator orch;
    global_orch = &orch;
    signal(SIGINT, sigint_handler);

    /* ---------- launch first MoveIt stack ---------- */
    orch.kill_all_and_wait();
    orch.launch(launch_cmd_for_gripper(start_gripper, robot_ip));
    
    // Wait for move_group node and ensure it's fully ready
    if (!orch.wait_for_node("move_group")) {
        RCLCPP_ERROR(node->get_logger(), "move_group node failed to start!");
        return 1;
    }
    
    // Wait for MoveIt to be fully ready instead of hardcoded sleep
    if (!wait_for_moveit_ready(node, std::chrono::seconds(30))) {
        RCLCPP_ERROR(node->get_logger(), "MoveIt failed to become ready!");
        return 1;
    }
    
    // Ensure the controller is active before proceeding
    if (!ensure_controller_active(node, std::chrono::seconds(30))) {
        RCLCPP_ERROR(node->get_logger(), "Failed to ensure controller is active!");
        return 1;
    }
    
    if (!play_dashboard_client(node)) {
        RCLCPP_WARN(node->get_logger(), "Dashboard play failed, but continuing - robot may already be running.");
    }
    
    if (!update_robot_description_from("move_group", node))
        return 1;

    orch.set_current_gripper(start_gripper);
    
    // Wait for robot to be in stable state instead of hardcoded sleep
    if (!wait_for_robot_stable(node, std::chrono::seconds(30))) {
        RCLCPP_WARN(node->get_logger(), "Robot did not reach stable state, continuing anyway...");
    }

    /* ---------- build MTC modules ---------- */
    PickPlaceStages    pick_place(node, cfg);
    ToolExchangeStages tool_exch(node, cfg);
    MoveToStages       moveto(node, cfg);
    EndEffectorStages  end_effector(node, cfg);

    /* =====================================================
     *  Main sequence loop
     * ===================================================== */
    for (const auto& step : sequence)
    {
        const std::string action = step.at("action");
        bool  success = false;

        if (action == "tool_exchange")
        {
            const std::string operation      = step.value("operation", "");
            const std::string requested_tool = step.value("gripper", orch.get_current_gripper());

            /* 1) Plan & execute the exchange with the CURRENT stack ----------------*/
            if (!update_robot_description_from("move_group", node) ||
                !tool_exch.run(step, poses, node))
                goto failed;

            /* 2) After successful motion, switch stacks if required ---------------*/
            if (operation == "dock")         // expecting to end with no tool attached
            {
                if (orch.get_current_gripper() == "none")
                    orch.set_current_gripper("none");    // already tool-less
                else {
                    orch.kill_all_and_wait();
                    orch.launch(launch_cmd_for_gripper("none", robot_ip));
                    if (!orch.wait_for_node("move_group") ||
                        !wait_for_moveit_ready(node, std::chrono::seconds(30)) ||
                        !ensure_controller_active(node, std::chrono::seconds(30)) ||
                        !update_robot_description_from("move_group", node))
                        goto failed;
                    if (!play_dashboard_client(node)) {
                        RCLCPP_WARN(node->get_logger(), "Dashboard play failed during tool exchange, but continuing...");
                    }
                    orch.set_current_gripper("none");
                }
            }
            else if (operation == "load")    // attach a new tool
            {
                if (orch.get_current_gripper() == requested_tool) {
                    // Already on correct tool – nothing to relaunch
                } else {
                    orch.kill_all_and_wait();
                    orch.launch(launch_cmd_for_gripper(requested_tool, robot_ip));
                    
                    if (!orch.wait_for_node("move_group") ||
                        !wait_for_moveit_ready(node, std::chrono::seconds(30)) ||
                        !ensure_controller_active(node, std::chrono::seconds(30)) ||
                        !update_robot_description_from("move_group", node))
                        goto failed;
                    if (!play_dashboard_client(node)) {
                        RCLCPP_WARN(node->get_logger(), "Dashboard play failed during tool load, but continuing...");
                    }
                    orch.set_current_gripper(requested_tool);
                }
            }
            success = true;
        }

        else if (action == "pick_and_place")
        {
            std::string need = step.value("gripper", orch.get_current_gripper());
            if (need != orch.get_current_gripper()) {
                orch.kill_all_and_wait();
                orch.launch(launch_cmd_for_gripper(need, robot_ip));
                if (!orch.wait_for_node("move_group") ||
                    !wait_for_moveit_ready(node, std::chrono::seconds(30)) ||
                    !ensure_controller_active(node, std::chrono::seconds(30)) ||
                    !update_robot_description_from("move_group", node))
                    goto failed;
                if (!play_dashboard_client(node)) {
                    RCLCPP_WARN(node->get_logger(), "Dashboard play failed during pick_and_place, but continuing...");
                }
                orch.set_current_gripper(need);
            }

            if (!update_robot_description_from("move_group", node) ||
                !pick_place.run(step, poses, node))
                goto failed;

            success = true;
        }

        else if (action == "moveto")
        {
            if (!update_robot_description_from("move_group", node) ||
                !moveto.run(step, poses, node))
                goto failed;

            success = true;
        }

        else if (action == "end_effector")
        {
            if (!end_effector.run(step, poses, node))
                goto failed;

            success = true;
        }

        if (!success) {
        failed:
            RCLCPP_ERROR(node->get_logger(), "%s step failed – aborting.", action.c_str());
            // Reduced failure timeout and added user prompt
            RCLCPP_INFO(node->get_logger(), "Press Enter to continue or Ctrl+C to exit...");
            std::cin.get();
            orch.kill_all_and_wait();
            rclcpp::shutdown();
            return 1;
        }
    }

    orch.kill_all_and_wait();
    rclcpp::shutdown();
    return 0;
}
