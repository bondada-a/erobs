// UR Tool Interface: Configure tool electrical interface via socket + dashboard service
//
// Extracted from MTCOrchestratorActionServer for better separation of concerns.
// Handles low-level communication with UR robot's secondary interface (port 30002)
// and dashboard service calls.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace mtc_pipeline {
namespace core {

class URToolInterface
{
public:
    URToolInterface(rclcpp::Node::SharedPtr node, const std::string& robot_ip);

    // Set robot IP (can be called later if not known at construction)
    void set_robot_ip(const std::string& robot_ip);

    // Set tool voltage via socket to UR secondary interface (port 30002)
    // Sends URScript: set_tool_voltage(voltage)
    // Returns true on success, false on failure
    bool set_tool_voltage(int voltage);

    // Restart UR external_control program via dashboard service
    // The voltage command stops the program, so we need to restart it
    // Returns true on success, false on failure
    bool restart_external_control();

private:
    rclcpp::Node::SharedPtr node_;
    std::string robot_ip_;
};

}  // namespace core
}  // namespace mtc_pipeline
