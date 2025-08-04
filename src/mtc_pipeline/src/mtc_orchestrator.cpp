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

/* ============================================================
 *  helpers
 * ============================================================
 */
void declare_if_needed(rclcpp::Node::SharedPtr node, const std::string& name)
{
    if (!node->has_parameter(name))
        node->declare_parameter(name, std::string{""});
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
    std::this_thread::sleep_for(std::chrono::seconds(15));
    auto client = node->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
    if (!client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "Dashboard 'play' service not available!");
        return false;
    }
    auto fut = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (rclcpp::spin_until_future_complete(node, fut, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS || !fut.get()->success) {
        RCLCPP_ERROR(node->get_logger(), "Dashboard 'play' call failed!");
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "Dashboard 'play' called.");
    return true;
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
        std::this_thread::sleep_for(std::chrono::seconds(10));

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
    auto node = rclcpp::Node::make_shared("mtc_orchestrator",
                                          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    Orchestrator orch; global_orch = &orch;
    signal(SIGINT, sigint_handler);

    /* ---------- read JSON script ---------- */
    std::string cfg_file = "./script.json";
    node->get_parameter("poses_file", cfg_file);
    std::ifstream ifs(cfg_file);
    if (!ifs) { RCLCPP_ERROR(node->get_logger(), "Cannot open %s", cfg_file.c_str()); return 1; }

    nlohmann::json cfg; ifs >> cfg;
    const auto& poses    = cfg.at("poses");
    const auto& sequence = cfg.at("sequence");

    /* ---------- parameters ---------- */
    std::string robot_ip = "192.168.1.101";
    node->get_parameter("robot_ip", robot_ip);

    std::string start_gripper = cfg.value("start_gripper", "none");

    /* ---------- launch first MoveIt stack ---------- */
    orch.kill_all_and_wait();
    orch.launch(launch_cmd_for_gripper(start_gripper, robot_ip));
    orch.wait_for_node("move_group");
    if (!play_dashboard_client(node) ||
        !update_robot_description_from("move_group", node))
        return 1;

    orch.set_current_gripper(start_gripper);
    std::this_thread::sleep_for(std::chrono::seconds(10));

    /* ---------- build MTC modules ---------- */
    PickPlaceStages    pick_place(node, cfg);
    ToolExchangeStages tool_exch(node, cfg);

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
                    orch.wait_for_node("move_group");
                    if (!play_dashboard_client(node) ||
                        !update_robot_description_from("move_group", node))
                        goto failed;
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
                    orch.wait_for_node("move_group");
                    if (!play_dashboard_client(node) ||
                        !update_robot_description_from("move_group", node))
                        goto failed;
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
                orch.wait_for_node("move_group");
                if (!play_dashboard_client(node) ||
                    !update_robot_description_from("move_group", node))
                    goto failed;
                orch.set_current_gripper(need);
            }

            if (!update_robot_description_from("move_group", node) ||
                !pick_place.run(step, poses, node))
                goto failed;

            success = true;
        }

        if (!success) {
        failed:
            RCLCPP_ERROR(node->get_logger(), "%s step failed – aborting.", action.c_str());
            std::this_thread::sleep_for(std::chrono::seconds(60));
            orch.kill_all_and_wait();
            rclcpp::shutdown();
            return 1;
        }
    }

    orch.kill_all_and_wait();
    rclcpp::shutdown();
    return 0;
}
