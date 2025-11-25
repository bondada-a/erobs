// MTC action client: sends task JSON to orchestrator and reports progress.

#include "mtc_pipeline/action/mtc_execution.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <fstream>
#include <chrono>
#include <optional>
#include <signal.h>
#include <atomic>

using namespace std::chrono_literals;
using MTCExecution = mtc_pipeline::action::MTCExecution;
using GoalHandleMTCExecution = rclcpp_action::ClientGoalHandle<MTCExecution>;

namespace {
std::atomic<bool> should_cancel{false};
}

class MTCActionClient : public rclcpp::Node {
public:
    enum class Result { Success = 0, Failure = 1, Cancelled = 2 };

    explicit MTCActionClient(std::chrono::seconds timeout = 300s)
        : Node("mtc_action_client"), timeout_(timeout) {
        action_client_ = rclcpp_action::create_client<MTCExecution>(this, "mtc_execution");
    }

    bool wait_for_server(std::chrono::seconds timeout = 10s) {
        return action_client_->wait_for_action_server(timeout);
    }

    int execute_task(const std::string& json_path, const std::string& robot_ip) {
        auto goal = create_goal(json_path, robot_ip);
        if (!goal) return static_cast<int>(Result::Failure);

        auto handle = send_goal(*goal);
        if (!handle) return static_cast<int>(Result::Failure);

        return static_cast<int>(wait_for_completion(handle));
    }

private:
    std::optional<MTCExecution::Goal> create_goal(const std::string& path, const std::string& ip) {
        try {
            std::ifstream file(path);
            MTCExecution::Goal goal;
            goal.full_json = std::string((std::istreambuf_iterator<char>(file)),
                                          std::istreambuf_iterator<char>());
            goal.robot_ip = ip;
            return goal;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error reading file: %s", e.what());
            return std::nullopt;
        }
    }

    std::shared_ptr<GoalHandleMTCExecution> send_goal(const MTCExecution::Goal& goal) {
        RCLCPP_INFO(get_logger(), "Sending goal...");

        auto options = rclcpp_action::Client<MTCExecution>::SendGoalOptions();
        options.feedback_callback = [this](auto, const auto& fb) {
            RCLCPP_INFO(get_logger(), "Progress: %.1f%% - %s - %s",
                fb->progress_percentage, fb->current_action.c_str(), fb->status_message.c_str());
        };
        options.result_callback = [this](const auto& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(get_logger(), "Task completed (%d/%d steps)",
                    result.result->completed_steps, result.result->total_steps);
            } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                RCLCPP_ERROR(get_logger(), "Aborted: %s", result.result->error_message.c_str());
            } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                RCLCPP_WARN(get_logger(), "Canceled");
            }
        };

        auto future = action_client_->async_send_goal(goal, options);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future, 10s) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed to send goal");
            return nullptr;
        }

        auto handle = future.get();
        if (!handle) {
            RCLCPP_ERROR(get_logger(), "Goal rejected");
            return nullptr;
        }
        RCLCPP_INFO(get_logger(), "Goal accepted");
        return handle;
    }

    Result wait_for_completion(std::shared_ptr<GoalHandleMTCExecution> handle) {
        auto future = action_client_->async_get_result(handle);
        auto start = std::chrono::steady_clock::now();

        while (true) {
            if (should_cancel.load()) {
                RCLCPP_INFO(get_logger(), "Cancelling...");
                action_client_->async_cancel_goal(handle);
                return Result::Cancelled;
            }

            if (rclcpp::spin_until_future_complete(shared_from_this(), future, 100ms) ==
                rclcpp::FutureReturnCode::SUCCESS) {
                auto result = future.get();
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) return Result::Success;
                if (result.code == rclcpp_action::ResultCode::CANCELED) return Result::Cancelled;
                return Result::Failure;
            }

            if (std::chrono::steady_clock::now() - start > timeout_) {
                RCLCPP_ERROR(get_logger(), "Timeout");
                return Result::Failure;
            }
        }
    }

    rclcpp_action::Client<MTCExecution>::SharedPtr action_client_;
    std::chrono::seconds timeout_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <json_file> [robot_ip] [timeout_sec]\n";
        return 1;
    }

    std::string json = argv[1];
    std::string ip = (argc > 2) ? argv[2] : "192.168.56.101";
    int timeout = (argc > 3) ? std::stoi(argv[3]) : 300;

    signal(SIGINT, [](int) { should_cancel.store(true); });

    auto client = std::make_shared<MTCActionClient>(std::chrono::seconds(timeout));

    RCLCPP_INFO(client->get_logger(), "Waiting for server...");
    if (!client->wait_for_server(10s)) {
        RCLCPP_ERROR(client->get_logger(), "Server unavailable");
        rclcpp::shutdown();
        return 1;
    }

    int code = client->execute_task(json, ip);
    rclcpp::shutdown();
    return code;
}
