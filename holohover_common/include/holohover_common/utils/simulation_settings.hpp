#ifndef HOLOHOVER_GNC_HOLOHOVER_SIMULATION_SETTINGS_HPP
#define HOLOHOVER_GNC_HOLOHOVER_SIMULATION_SETTINGS_HPP

struct SimulationSettings
{
    double period;
    int seed;

    double hovercraft_radius;
    int internal_iterations_velocity;
    int internal_iterations_position;

    bool are_all_simulated;
    std::vector<long int> hovercraft_ids;
    std::vector<std::string> hovercraft_names;
    std::vector<std::string> holohover_props_files;
    std::vector<double>   start_position_x;
    std::vector<double>   start_position_y;
    std::vector<double>   start_position_theta;
    std::vector<double>   start_position_vx;
    std::vector<double>   start_position_vy;
    std::vector<double>   start_position_w;

    std::vector<double> table_size;
    std::vector<double> table_position;
    double table_publish_period;

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


    settings.hovercraft_radius = node.declare_parameter<double>("hovercraft_radius");
    settings.internal_iterations_velocity = node.declare_parameter<int>("internal_iterations_velocity");
    settings.internal_iterations_position = node.declare_parameter<int>("internal_iterations_position");

    settings.are_all_simulated    = node.declare_parameter<bool>("are_all_simulated");
    settings.hovercraft_ids       = node.declare_parameter<std::vector<long int>>("hovercraft_ids");
    settings.hovercraft_names     = node.declare_parameter<std::vector<std::string>>("hovercraft_names");
    settings.holohover_props_files= node.declare_parameter<std::vector<std::string>>("holohover_props_files");
    settings.start_position_x     = node.declare_parameter<std::vector<double>>("initial_state_x");
    settings.start_position_y     = node.declare_parameter<std::vector<double>>("initial_state_y");
    settings.start_position_theta = node.declare_parameter<std::vector<double>>("initial_state_theta");
    settings.start_position_vx    = node.declare_parameter<std::vector<double>>("initial_state_vx");
    settings.start_position_vy    = node.declare_parameter<std::vector<double>>("initial_state_vy");
    settings.start_position_w     = node.declare_parameter<std::vector<double>>("initial_state_w");

    settings.table_size           = node.declare_parameter<std::vector<double>>("table_size");
    settings.table_position       = node.declare_parameter<std::vector<double>>("table_position");
    settings.table_publish_period = node.declare_parameter<double>("table_publish_period");

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
