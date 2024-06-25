#ifndef HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_SETTINGS_HPP
#define HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_SETTINGS_HPP

struct NavigationSettings
{
    double period;

    double state_cov_x;
    double state_cov_y;
    double state_cov_yaw;
    double state_cov_x_dot;
    double state_cov_y_dot;
    double state_cov_yaw_dot;
    double state_cov_x_dot_dot;
    double state_cov_y_dot_dot;
    double state_cov_yaw_dot_dot;
    double state_cov_dist_x;
    double state_cov_dist_y;
    double state_cov_dist_yaw;

    double control_cov_a_x;
    double control_cov_a_y;
    double control_cov_w_dot_z;

    double sensor_acc_cov_x;
    double sensor_acc_cov_y;

    double sensor_gyro_cov_z;

    double sensor_mouse_cov_x;
    double sensor_mouse_cov_y;

    double sensor_pose_cov_x;
    double sensor_pose_cov_y;
    double sensor_pose_cov_yaw;
};

NavigationSettings load_navigation_settings(rclcpp::Node &node)
{
    NavigationSettings settings;

    settings.period = node.declare_parameter<double>("period");

    settings.state_cov_x = node.declare_parameter<double>("state_cov_x");
    settings.state_cov_y = node.declare_parameter<double>("state_cov_y");
    settings.state_cov_yaw = node.declare_parameter<double>("state_cov_yaw");
    settings.state_cov_x_dot = node.declare_parameter<double>("state_cov_x_dot");
    settings.state_cov_y_dot = node.declare_parameter<double>("state_cov_y_dot");
    settings.state_cov_yaw_dot = node.declare_parameter<double>("state_cov_yaw_dot");
    settings.state_cov_x_dot_dot = node.declare_parameter<double>("state_cov_x_dot_dot");
    settings.state_cov_y_dot_dot = node.declare_parameter<double>("state_cov_y_dot_dot");
    settings.state_cov_yaw_dot_dot = node.declare_parameter<double>("state_cov_yaw_dot_dot");
    settings.state_cov_dist_x = node.declare_parameter<double>("state_cov_dist_x");
    settings.state_cov_dist_y = node.declare_parameter<double>("state_cov_dist_y");
    settings.state_cov_dist_yaw = node.declare_parameter<double>("state_cov_dist_yaw");

    settings.control_cov_a_x = node.declare_parameter<double>("control_cov_a_x");
    settings.control_cov_a_y = node.declare_parameter<double>("control_cov_a_y");
    settings.control_cov_w_dot_z = node.declare_parameter<double>("control_cov_w_dot_z");

    settings.sensor_acc_cov_x = node.declare_parameter<double>("sensor_acc_cov_x");
    settings.sensor_acc_cov_y = node.declare_parameter<double>("sensor_acc_cov_y");

    settings.sensor_gyro_cov_z = node.declare_parameter<double>("sensor_gyro_cov_z");

    settings.sensor_mouse_cov_x = node.declare_parameter<double>("sensor_mouse_cov_x");
    settings.sensor_mouse_cov_y = node.declare_parameter<double>("sensor_mouse_cov_y");

    settings.sensor_pose_cov_x = node.declare_parameter<double>("sensor_pose_cov_x");
    settings.sensor_pose_cov_y = node.declare_parameter<double>("sensor_pose_cov_y");
    settings.sensor_pose_cov_yaw = node.declare_parameter<double>("sensor_pose_cov_yaw");

    return settings;
}

#endif //HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_SETTINGS_HPP
