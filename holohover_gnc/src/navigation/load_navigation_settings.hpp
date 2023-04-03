#ifndef HOLOHOVER_GNC_HOLOHOVER_LOAD_NAVIGATION_SETTINGS_HPP
#define HOLOHOVER_GNC_HOLOHOVER_LOAD_NAVIGATION_SETTINGS_HPP

#include "rclcpp/rclcpp.hpp"
#include "navigation_settings.hpp"

NavigationSettings load_navigation_settings(rclcpp::Node &node)
{
    NavigationSettings settings;

    if (node.get_parameter("period", settings.period) &&
        node.get_parameter("state_cov_x", settings.state_cov_x) &&
        node.get_parameter("state_cov_y", settings.state_cov_y) &&
        node.get_parameter("state_cov_yaw", settings.state_cov_yaw) &&
        node.get_parameter("state_cov_x_dot", settings.state_cov_x_dot) &&
        node.get_parameter("state_cov_y_dot", settings.state_cov_y_dot) &&
        node.get_parameter("state_cov_yaw_dot", settings.state_cov_yaw_dot) &&
        node.get_parameter("state_cov_x_dot_dot", settings.state_cov_x_dot_dot) &&
        node.get_parameter("state_cov_y_dot_dot", settings.state_cov_y_dot_dot) &&
        node.get_parameter("state_cov_yaw_dot_dot", settings.state_cov_yaw_dot_dot) &&
        node.get_parameter("control_cov_a_x", settings.control_cov_a_x) &&
        node.get_parameter("control_cov_a_y", settings.control_cov_a_y) &&
        node.get_parameter("control_cov_w_dot_z", settings.control_cov_w_dot_z) &&
        node.get_parameter("sensor_acc_cov_x", settings.sensor_acc_cov_x) &&
        node.get_parameter("sensor_acc_cov_y", settings.sensor_acc_cov_y) &&
        node.get_parameter("sensor_gyro_cov_z", settings.sensor_gyro_cov_z) &&
        node.get_parameter("sensor_mouse_cov_x", settings.sensor_mouse_cov_x) &&
        node.get_parameter("sensor_mouse_cov_y", settings.sensor_mouse_cov_y) &&
        node.get_parameter("sensor_pose_cov_x", settings.sensor_pose_cov_x) &&
        node.get_parameter("sensor_pose_cov_y", settings.sensor_pose_cov_y) &&
        node.get_parameter("sensor_pose_cov_yaw", settings.sensor_pose_cov_yaw)) {}
    else
    {
        RCLCPP_INFO(node.get_logger(), "Failed to load navigation settings");
    }

    return settings;
}

#endif //HOLOHOVER_GNC_HOLOHOVER_LOAD_NAVIGATION_SETTINGS_HPP
