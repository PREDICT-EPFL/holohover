#ifndef HOLOHOVER_GNC_HOLOHOVER_SIMULATION_SETTINGS_HPP
#define HOLOHOVER_GNC_HOLOHOVER_SIMULATION_SETTINGS_HPP

struct SimulationSettings
{
    double period;
    int seed;

    double drag_reference_area;
    double drag_coefficient;

    double table_tilt_x;
    double table_tilt_y;

    double sensor_imu_period;
    double sensor_acc_noise_x;
    double sensor_acc_noise_y;
    double sensor_acc_bias_x;
    double sensor_acc_bias_y;
    double sensor_gyro_noise_z;
    double sensor_gyro_bias_z;

    double sensor_mouse_period;
    double sensor_mouse_noise_x;
    double sensor_mouse_noise_y;

    double sensor_pose_period;
    double sensor_pose_noise_x;
    double sensor_pose_noise_y;
    double sensor_pose_noise_yaw;

    double Gx;
    double Gy;
};

SimulationSettings load_simulation_settings(rclcpp::Node &node)
{
    SimulationSettings settings;

    settings.period = node.declare_parameter<double>("period");
    settings.seed = node.declare_parameter<int>("seed");

    settings.drag_reference_area = node.declare_parameter<double>("drag_reference_area");
    settings.drag_coefficient = node.declare_parameter<double>("drag_coefficient");

    settings.table_tilt_x = node.declare_parameter<double>("table_tilt_x");
    settings.table_tilt_y = node.declare_parameter<double>("table_tilt_y");

    settings.sensor_imu_period = node.declare_parameter<double>("sensor_imu_period");
    settings.sensor_acc_noise_x = node.declare_parameter<double>("sensor_acc_noise_x");
    settings.sensor_acc_noise_y = node.declare_parameter<double>("sensor_acc_noise_y");
    settings.sensor_acc_bias_x = node.declare_parameter<double>("sensor_acc_bias_x");
    settings.sensor_acc_bias_y = node.declare_parameter<double>("sensor_acc_bias_y");
    settings.sensor_gyro_noise_z = node.declare_parameter<double>("sensor_gyro_noise_z");
    settings.sensor_gyro_bias_z = node.declare_parameter<double>("sensor_gyro_bias_z");

    settings.sensor_mouse_period = node.declare_parameter<double>("sensor_mouse_period");
    settings.sensor_mouse_noise_x = node.declare_parameter<double>("sensor_mouse_noise_x");
    settings.sensor_mouse_noise_y = node.declare_parameter<double>("sensor_mouse_noise_y");

    settings.sensor_pose_period = node.declare_parameter<double>("sensor_pose_period");
    settings.sensor_pose_noise_x = node.declare_parameter<double>("sensor_pose_noise_x");
    settings.sensor_pose_noise_y = node.declare_parameter<double>("sensor_pose_noise_y");
    settings.sensor_pose_noise_yaw = node.declare_parameter<double>("sensor_pose_noise_yaw");

    settings.Gx = node.declare_parameter<double>("Gx");
    settings.Gy = node.declare_parameter<double>("Gy");

    return settings;
}

#endif //HOLOHOVER_GNC_HOLOHOVER_SIMULATION_SETTINGS_HPP
