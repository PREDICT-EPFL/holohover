#ifndef HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_SETTINGS_HPP
#define HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_SETTINGS_HPP

struct NavigationSettings
{
    double period;

    double state_cov_x;
    double state_cov_y;
    double state_cov_v_x;
    double state_cov_v_y;
    double state_cov_yaw;
    double state_cov_w_z;

    double control_cov_a_x;
    double control_cov_a_y;
    double control_cov_w_dot_z;

    double sensor_acc_cov_a_x_body;
    double sensor_acc_cov_a_y_body;

    double sensor_gyro_cov_w_z;

    double sensor_mouse_cov_v_x_body;
    double sensor_mouse_cov_v_y_body;

    double sensor_pose_cov_x;
    double sensor_pose_cov_y;
    double sensor_pose_cov_yaw;
};

#endif //HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_SETTINGS_HPP
