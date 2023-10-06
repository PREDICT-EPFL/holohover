#ifndef HOLOHOVER_GNC_HOLOHOVER_CONTROL_EXP_SETTINGS_HPP
#define HOLOHOVER_GNC_HOLOHOVER_CONTROL_EXP_SETTINGS_HPP

#include "rclcpp/rclcpp.hpp"

struct ControlExpSettings
{   
    double rand_sig_duration;
    double rand_sig_max;
    double rand_sig_min;
};

ControlExpSettings load_control_exp_settings(rclcpp::Node &node)
{
    ControlExpSettings settings;

    settings.rand_sig_duration = node.declare_parameter<double>("rand_sig_duration");
    settings.rand_sig_max = node.declare_parameter<double>("rand_sig_max");
    settings.rand_sig_min = node.declare_parameter<double>("rand_sig_min");

    return settings;
}

#endif //HOLOHOVER_GNC_HOLOHOVER_CONTROL_EXP_SETTINGS_HPP