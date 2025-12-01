// Manages UR robot tool voltage and external_control program lifecycle.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace mtc_pipeline::core {

class URToolInterface
{
public:
    /// @brief Construct UR tool interface with ROS 2 node and robot IP
    URToolInterface(rclcpp::Node* node, const std::string& robot_ip);

    /// @brief Update robot IP address for socket connections
    void set_robot_ip(const std::string& robot_ip);

    /// @brief Set tool output voltage via URScript socket command
    bool set_tool_voltage(int voltage);

    /// @brief Restart external_control program via dashboard service
    bool restart_external_control();

private:
    rclcpp::Node* node_;
    std::string robot_ip_;
};

}
