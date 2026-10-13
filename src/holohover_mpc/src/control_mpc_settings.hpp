#ifndef HOLOHOVER_GNC_CONTROL_MPC_SETTINGS_HPP
#define HOLOHOVER_GNC_CONTROL_MPC_SETTINGS_HPP

#include "rclcpp/rclcpp.hpp"

struct ControlMPCSettings
{
    double period;

    double weight_x;
    double weight_y;
    double weight_v_x;
    double weight_v_y;
    double weight_yaw;
    double weight_w_z;

    double weight_motor;
};

ControlMPCSettings load_control_mpc_settings(rclcpp::Node &node)
{
    ControlMPCSettings settings;
    
    settings.period = node.declare_parameter<double>("period");

    settings.weight_x = node.declare_parameter<double>("weight_x");
    settings.weight_y = node.declare_parameter<double>("weight_y");
    settings.weight_v_x = node.declare_parameter<double>("weight_v_x");
    settings.weight_v_y = node.declare_parameter<double>("weight_v_y");
    settings.weight_yaw = node.declare_parameter<double>("weight_yaw");
    settings.weight_w_z = node.declare_parameter<double>("weight_w_z");

    settings.weight_motor = node.declare_parameter<double>("weight_motor");

    return settings;
}

#endif //HOLOHOVER_GNC_CONTROL_MPC_SETTINGS_HPP