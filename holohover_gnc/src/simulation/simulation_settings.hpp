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

#endif //HOLOHOVER_GNC_HOLOHOVER_SIMULATION_SETTINGS_HPP
