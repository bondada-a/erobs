// Orchestrator that dispatches tasks to specialized action servers

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <hello_orchestrator/action/orchestrator_task.hpp>
#include <hello_orchestrator/action/print_message.hpp>
#include <hello_orchestrator/action/move_to_named.hpp>

using OrchestratorTask = hello_orchestrator::action::OrchestratorTask;
using PrintMessage = hello_orchestrator::action::PrintMessage;
using MoveToNamed = hello_orchestrator::action::MoveToNamed;
using GoalHandleOrchestrator = rclcpp_action::ServerGoalHandle<OrchestratorTask>;

class OrchestratorServer : public rclcpp::Node
{
public:
    OrchestratorServer() : Node("orchestrator_server")
    {
        // Create action clients to specialized servers
        print_client_ = rclcpp_action::create_client<PrintMessage>(this, "print_message");
        move_client_ = rclcpp_action::create_client<MoveToNamed>(this, "move_to_named");

        // Create orchestrator action server
        action_server_ = rclcpp_action::create_server<OrchestratorTask>(
            this,
            "orchestrator_task",
            std::bind(&OrchestratorServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&OrchestratorServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&OrchestratorServer::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Orchestrator action server started");
        RCLCPP_INFO(this->get_logger(), "Waiting for print_message and move_to_named servers...");
    }

private:
    rclcpp_action::Server<OrchestratorTask>::SharedPtr action_server_;
    rclcpp_action::Client<PrintMessage>::SharedPtr print_client_;
    rclcpp_action::Client<MoveToNamed>::SharedPtr move_client_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const OrchestratorTask::Goal> /*goal*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received orchestrator goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleOrchestrator> /*goal_handle*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleOrchestrator> goal_handle)
    {
        std::thread{[this, goal_handle]() {
            execute(goal_handle);
        }}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleOrchestrator> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing orchestrator goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<OrchestratorTask::Feedback>();
        auto result = std::make_shared<OrchestratorTask::Result>();

        // Parse JSON task
        nlohmann::json task_json;
        try {
            task_json = nlohmann::json::parse(goal->task_json);
        } catch (const nlohmann::json::exception& e) {
            result->success = false;
            result->error_message = std::string("JSON parse error: ") + e.what();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->error_message.c_str());
            return;
        }

        if (!task_json.contains("tasks") || !task_json["tasks"].is_array()) {
            result->success = false;
            result->error_message = "JSON must contain 'tasks' array";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->error_message.c_str());
            return;
        }

        auto tasks = task_json["tasks"];
        feedback->total_steps = tasks.size();

        RCLCPP_INFO(this->get_logger(), "📋 ORCHESTRATOR: Executing %zu tasks", tasks.size());

        // Execute each task
        for (size_t i = 0; i < tasks.size(); ++i) {
            const auto& task = tasks[i];

            if (!task.contains("type")) {
                result->success = false;
                result->error_message = "Task " + std::to_string(i) + " missing 'type' field";
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "%s", result->error_message.c_str());
                return;
            }

            std::string type = task["type"];
            feedback->current_step = i + 1;
            feedback->current_action = type;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "  → Step %zu/%zu: %s",
                        i + 1, tasks.size(), type.c_str());

            // Dispatch to appropriate server
            bool success = false;
            if (type == "print") {
                success = execute_print_task(task);
            } else if (type == "move") {
                success = execute_move_task(task);
            } else {
                result->success = false;
                result->error_message = "Unknown task type: " + type;
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "%s", result->error_message.c_str());
                return;
            }

            if (!success) {
                result->success = false;
                result->error_message = "Task " + std::to_string(i) + " (" + type + ") failed";
                result->completed_steps = i;
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "%s", result->error_message.c_str());
                return;
            }

            result->completed_steps = i + 1;
        }

        // All tasks completed successfully
        result->success = true;
        result->error_message = "";
        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(), "✅ ORCHESTRATOR: All tasks completed successfully");
    }

    bool execute_print_task(const nlohmann::json& task)
    {
        if (!task.contains("message")) {
            RCLCPP_ERROR(this->get_logger(), "Print task missing 'message' field");
            return false;
        }

        if (!print_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Print action server not available");
            return false;
        }

        auto goal = PrintMessage::Goal();
        goal.message = task["message"];

        auto goal_handle = print_client_->async_send_goal(goal).get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Print goal rejected");
            return false;
        }

        auto result_future = print_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Print action timeout");
            return false;
        }

        auto result = result_future.get();
        return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
    }

    bool execute_move_task(const nlohmann::json& task)
    {
        if (!task.contains("target")) {
            RCLCPP_ERROR(this->get_logger(), "Move task missing 'target' field");
            return false;
        }

        if (!move_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Move action server not available");
            return false;
        }

        auto goal = MoveToNamed::Goal();
        goal.target_pose = task["target"];

        auto goal_handle = move_client_->async_send_goal(goal).get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Move goal rejected");
            return false;
        }

        auto result_future = move_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Move action timeout");
            return false;
        }

        auto result = result_future.get();
        return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrchestratorServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
