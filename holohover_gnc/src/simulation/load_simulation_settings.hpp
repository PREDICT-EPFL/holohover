#ifndef HOLOHOVER_GNC_HOLOHOVER_LOAD_NAVIGATION_SETTINGS_HPP
#define HOLOHOVER_GNC_HOLOHOVER_LOAD_NAVIGATION_SETTINGS_HPP

#include "rclcpp/rclcpp.hpp"
#include "simulation_settings.hpp"

SimulationSettings load_simulation_settings(rclcpp::Node &node)
{
    SimulationSettings settings;

    if (node.get_parameter("period", settings.period) &&
        node.get_parameter("seed", settings.seed) &&
        node.get_parameter("drag_reference_area", settings.drag_reference_area) &&
        node.get_parameter("drag_coefficient", settings.drag_coefficient) &&
        node.get_parameter("table_tilt_x", settings.table_tilt_x) &&
        node.get_parameter("table_tilt_y", settings.table_tilt_y) &&
        node.get_parameter("sensor_imu_period", settings.sensor_imu_period) &&
        node.get_parameter("sensor_acc_noise_x", settings.sensor_acc_noise_x) &&
        node.get_parameter("sensor_acc_noise_y", settings.sensor_acc_noise_y) &&
        node.get_parameter("sensor_acc_bias_x", settings.sensor_acc_bias_x) &&
        node.get_parameter("sensor_acc_bias_y", settings.sensor_acc_bias_y) &&
        node.get_parameter("sensor_gyro_noise_z", settings.sensor_gyro_noise_z) &&
        node.get_parameter("sensor_gyro_bias_z", settings.sensor_gyro_bias_z) &&
        node.get_parameter("sensor_mouse_period", settings.sensor_mouse_period) &&
        node.get_parameter("sensor_mouse_noise_x", settings.sensor_mouse_noise_x) &&
        node.get_parameter("sensor_mouse_noise_y", settings.sensor_mouse_noise_y) &&
        node.get_parameter("sensor_pose_period", settings.sensor_pose_period) &&
        node.get_parameter("sensor_pose_noise_x", settings.sensor_pose_noise_x) &&
        node.get_parameter("sensor_pose_noise_y", settings.sensor_pose_noise_y) &&
        node.get_parameter("sensor_pose_noise_yaw", settings.sensor_pose_noise_yaw)) {}
    else
    {
        RCLCPP_INFO(node.get_logger(), "Failed to load navigation settings");
    }

    return settings;
}

#endif //HOLOHOVER_GNC_HOLOHOVER_LOAD_NAVIGATION_SETTINGS_HPP
