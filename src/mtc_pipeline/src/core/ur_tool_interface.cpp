/**
 * UR Tool Interface Implementation
 *
 * Extracted from MTCOrchestratorActionServer (lines 435-470 + 365-368)
 * All logic preserved exactly as-is for behavior compatibility.
 */

#include "mtc_pipeline/core/ur_tool_interface.hpp"

using namespace std::chrono_literals;

namespace mtc_pipeline {
namespace core {

URToolInterface::URToolInterface(rclcpp::Node::SharedPtr node, const std::string& robot_ip)
    : node_(node), robot_ip_(robot_ip)
{
}

void URToolInterface::set_robot_ip(const std::string& robot_ip)
{
    robot_ip_ = robot_ip;
}

bool URToolInterface::set_tool_voltage(int voltage)
{
    // Uses raw socket because this runs BEFORE MoveIt/ROS services are available
    // (EXACT copy from orchestrator lines 435-470)

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create socket");
        return false;
    }

    // 2-second timeout
    struct timeval timeout = {2, 0};
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    // Connect to UR secondary interface (port 30002)
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(30002);
    inet_pton(AF_INET, robot_ip_.c_str(), &addr.sin_addr);

    if (connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(sockfd);
        RCLCPP_ERROR(node_->get_logger(), "Failed to connect to %s:30002", robot_ip_.c_str());
        return false;
    }

    // Send URScript command
    std::string cmd = "set_tool_voltage(" + std::to_string(voltage) + ")\n";
    bool success = send(sockfd, cmd.c_str(), cmd.length(), 0) > 0;
    close(sockfd);

    return success;
}

bool URToolInterface::restart_external_control()
{
    // Restart UR external_control program (voltage command stops it)
    // (EXACT copy from orchestrator lines 365-368)

    auto dashboard = node_->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
    dashboard->wait_for_service(30s);
    dashboard->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    return true;
}

}  // namespace core
}  // namespace mtc_pipeline
