#ifndef HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_SETTINGS_HPP
#define HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_SETTINGS_HPP

#include "rclcpp/rclcpp.hpp"

struct ControlLQRSettings
{
    double period;

    double weight_x;
    double weight_y;
    double weight_v_x;
    double weight_v_y;
    double weight_yaw;
    double weight_w_z;

    double weight_a_x;
    double weight_a_y;
    double weight_w_dot_z;

    double initial_x;
    double initial_y;
    double initial_yaw;
};

ControlLQRSettings load_control_lqr_settings(rclcpp::Node &node)
{
    ControlLQRSettings settings;

    settings.period = node.declare_parameter<double>("period");

    settings.weight_x = node.declare_parameter<double>("weight_x");
    settings.weight_y = node.declare_parameter<double>("weight_y");
    settings.weight_v_x = node.declare_parameter<double>("weight_v_x");
    settings.weight_v_y = node.declare_parameter<double>("weight_v_y");
    settings.weight_yaw = node.declare_parameter<double>("weight_yaw");
    settings.weight_w_z = node.declare_parameter<double>("weight_w_z");

    settings.weight_a_x = node.declare_parameter<double>("weight_a_x");
    settings.weight_a_y = node.declare_parameter<double>("weight_a_y");
    settings.weight_w_dot_z = node.declare_parameter<double>("weight_w_dot_z");

    settings.initial_x = node.declare_parameter<double>("initial_x");
    settings.initial_y = node.declare_parameter<double>("initial_y");
    settings.initial_yaw = node.declare_parameter<double>("initial_yaw");

    return settings;
}

#endif //HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_SETTINGS_HPP