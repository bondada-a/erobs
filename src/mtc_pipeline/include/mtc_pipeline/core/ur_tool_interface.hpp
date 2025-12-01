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
    URToolInterface(rclcpp::Node* node, const std::string& robot_ip);

    void set_robot_ip(const std::string& robot_ip);

    // Sends URScript command via socket to port 30002
    bool set_tool_voltage(int voltage);

    // Voltage command stops external_control, this restarts it via dashboard
    bool restart_external_control();

private:
    rclcpp::Node* node_;
    std::string robot_ip_;
};

}
