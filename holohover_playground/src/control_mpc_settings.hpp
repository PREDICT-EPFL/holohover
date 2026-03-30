#ifndef HOLOHOVER_GNC_CONTROL_MPC_SETTINGS_HPP
#define HOLOHOVER_GNC_CONTROL_MPC_SETTINGS_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>

struct ControlMPCSettings
{
    double period;
    std::string solver;

    double weight_x;
    double weight_y;
    double weight_v_x;
    double weight_v_y;
    double weight_yaw;
    double weight_w_z;

    double weight_distance;
    double weight_momentum;
    double weight_comehome;
    double scale_distance;
    double scale_momentum;
    double weight_motor;

    double home_x;
    double home_y;
    double goal_x;
    double goal_y;

    double control_limit;
    double controller_delay;
};

ControlMPCSettings load_control_mpc_settings(rclcpp::Node &node)
{
    ControlMPCSettings settings;
    
    settings.period = node.declare_parameter<double>("period");
    settings.solver = node.declare_parameter<std::string>("solver", "ipopt");
    settings.weight_x = node.declare_parameter<double>("weight_x");
    settings.weight_y = node.declare_parameter<double>("weight_y");
    settings.weight_v_x = node.declare_parameter<double>("weight_v_x");
    settings.weight_v_y = node.declare_parameter<double>("weight_v_y");
    settings.weight_yaw = node.declare_parameter<double>("weight_yaw");
    settings.weight_w_z = node.declare_parameter<double>("weight_w_z");

    settings.weight_motor = node.declare_parameter<double>("weight_motor");
    settings.weight_momentum = node.declare_parameter<double>("weight_momentum");
    settings.weight_comehome = node.declare_parameter<double>("weight_comehome");
    settings.weight_distance = node.declare_parameter<double>("weight_distance");
    settings.scale_distance = node.declare_parameter<double>("scale_distance");
    settings.scale_momentum = node.declare_parameter<double>("scale_momentum");

    settings.home_x = node.declare_parameter<double>("home_x");
    settings.home_y = node.declare_parameter<double>("home_y");
    settings.goal_x = node.declare_parameter<double>("goal_x");
    settings.goal_y = node.declare_parameter<double>("goal_y");

    settings.control_limit = node.declare_parameter<double>("control_limit");
    settings.controller_delay = node.declare_parameter<double>("controller_delay");
    return settings;
}

#endif //HOLOHOVER_GNC_CONTROL_MPC_SETTINGS_HPP