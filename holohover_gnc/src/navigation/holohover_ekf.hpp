#ifndef HOLOHOVER_GNC_HOLOHOVER_EKF_HPP
#define HOLOHOVER_GNC_HOLOHOVER_EKF_HPP

#include "holohover_gnc/models/holohover_model.hpp"
#include "navigation_settings.hpp"

class HolohoverEKF {
public:
    static const int NX = Holohover::NX;
    static const int NU = Holohover::NU;
    static const int NA = Holohover::NA;

    // x = (x, y, v_x, v_y, yaw, w_z)
    using state_t = Eigen::Matrix<double, NX, 1>;
    using state_matrix_t = Eigen::Matrix<double, NX, NX>;

    // u = (F_1, F_2, F_3, F_4, F_5, F_6)
    using control_force_t = Eigen::Matrix<double, NU, 1>;

    // u = (a_x, a_y, w_dot_z)
    using control_acc_t = Eigen::Matrix<double, NA, 1>;
    using control_acc_matrix_t = Eigen::Matrix<double, NA, NA>;

    // z = (a_x_body, a_y_body, w_z)
    using sensor_imu_t = Eigen::Matrix<double, 3, 1>;
    using sensor_acc_matrix_t = Eigen::Matrix<double, 2, 2>;
    using sensor_gyro_matrix_t = Eigen::Matrix<double, 1, 1>;

    // z = (v_x_body, v_y_body)
    using sensor_mouse_t = Eigen::Matrix<double, 2, 1>;
    using sensor_mouse_matrix_t = Eigen::Matrix<double, 2, 2>;

    // z = (x, y, yaw)
    using sensor_pose_t = Eigen::Matrix<double, 3, 1>;
    using sensor_pose_matrix_t = Eigen::Matrix<double, 3, 3>;

    const Holohover holohover;
    const NavigationSettings settings;

    // state noise covariance
    state_matrix_t Q;
    // control input noise covariance
    control_acc_matrix_t Q_control;
    // accelerometer sensor noise covariance
    sensor_acc_matrix_t Q_acc;
    // gyroscope sensor noise covariance
    sensor_gyro_matrix_t R_gyro;
    // mouse speed sensor noise covariance
    sensor_mouse_matrix_t R_mouse;
    // pose sensor noise covariance
    sensor_pose_matrix_t R_pose;

    // predicted state estimate
    state_t x;
    // predicted state covariance estimate
    state_matrix_t P;

    HolohoverEKF(const Holohover &_holohover, const NavigationSettings &_settings) :
            holohover(_holohover), settings(_settings)
    {
        Q.setIdentity();
        Q.diagonal() << settings.state_cov_x, settings.state_cov_y,
                        settings.state_cov_v_x, settings.state_cov_v_y,
                        settings.state_cov_yaw, settings.state_cov_w_z;

        Q_control.setIdentity();
        Q_control.diagonal() << settings.control_cov_a_x, settings.control_cov_a_y, settings.control_cov_w_dot_z;

        Q_acc.setIdentity();
        Q_acc.diagonal() << settings.sensor_acc_cov_x, settings.sensor_acc_cov_y;

        R_gyro.setIdentity();
        R_gyro.diagonal() << settings.sensor_gyro_cov_z;

        R_mouse.setIdentity();
        R_mouse.diagonal() << settings.sensor_mouse_cov_x, settings.sensor_mouse_cov_y;

        R_pose.setIdentity();
        R_pose.diagonal() << settings.sensor_pose_cov_x, settings.sensor_pose_cov_y, settings.sensor_pose_cov_yaw;

        // init estimate
        x.setZero();
        P = Q;
    }

    void predict_integrate(const control_acc_t &u_acc, const state_matrix_t &Q_full)
    {
        // integrate state estimate x using discretized dynamics
        x = holohover.Ad * x + holohover.Bd * u_acc;

        // integrate covariance estimate P using RK4
        double dt = settings.period;
        auto P_dot = [&] (const state_matrix_t &Pt)
        {
            return holohover.A * Pt + Pt * holohover.A.transpose() + Q_full;
        };
        state_matrix_t P_k1 = P_dot(P);
        state_matrix_t P_k2 = P_dot(P + dt * P_k1 / 2);
        state_matrix_t P_k3 = P_dot(P + dt * P_k2 / 2);
        state_matrix_t P_k4 = P_dot(P + dt * P_k3);
        P += (P_k1 + 2 * P_k2 + 2 * P_k3 + P_k4) * dt / 6;
    }

    void predict_control_acc(const control_acc_t &u_acc)
    {
        state_matrix_t Q_full = Q + holohover.Bd * Q_control * holohover.Bd.transpose();
        predict_integrate(u_acc, Q_full);
    }

    void predict_control_force(const control_force_t &u_force)
    {
        control_acc_t u_acc;
        holohover.control_force_to_acceleration(x, u_force, u_acc);
        predict_control_acc(u_acc);
    }

    void predict_sensor_acc(const sensor_imu_t &imu_sensor)
    {
        Eigen::Matrix<double, 2, 2> rotation_matrix;
        holohover.body_to_world_rotation_matrix(x, rotation_matrix);

        // transforms acc measurement into control acc
        Eigen::Matrix<double, 3, 2> transform;
        transform.setZero();
        transform.topLeftCorner<2, 2>() = rotation_matrix;

        control_acc_t u_acc = transform * imu_sensor.head<2>();
        state_matrix_t Q_full = Q + holohover.Bd * transform * Q_acc * transform.transpose() * holohover.Bd.transpose();
        predict_integrate(u_acc, Q_full);
    }

    void predict_control_acc_and_sensor_acc(const control_acc_t &u_acc, const sensor_imu_t &imu_sensor)
    {
        Eigen::Matrix<double, 2, 2> rotation_matrix;
        holohover.world_to_body_rotation_matrix(x, rotation_matrix);

        // use kalman update step to incorporate sensor information
        // measurement model
        Eigen::Matrix<double, 2, 3> H;
        H.setZero();
        H.topLeftCorner<2, 2>() = rotation_matrix;
        // innovation covariance
        Eigen::Matrix<double, 2, 2> S = H * Q_control * H.transpose() + Q_acc;
        // kalman gain
        Eigen::Matrix<double, 3, 2> K = S.transpose().ldlt().solve(H * Q_control.transpose()).transpose();
        // update control acc estimate
        control_acc_t u_acc_estimate = u_acc + K * (imu_sensor.head<2>() - H * u_acc);
        // update covariance estimate
        control_acc_matrix_t Q_control_estimate = (control_acc_matrix_t::Identity() - K * H) * Q_control;

        state_matrix_t Q_full = Q + holohover.Bd * Q_control_estimate * holohover.Bd.transpose();
        predict_integrate(u_acc_estimate, Q_full);
    }

    void predict_control_force_and_sensor_acc(const control_force_t &u_force, const sensor_imu_t &imu_sensor)
    {
        control_acc_t u_acc;
        holohover.control_force_to_acceleration(x, u_force, u_acc);
        predict_control_acc_and_sensor_acc(u_acc, imu_sensor);
    }

    template<int NZ>
    void update_generic(const Eigen::Matrix<double, NZ, 1> &z,
                        const Eigen::Matrix<double, NZ, NX> &h,
                        const Eigen::Matrix<double, NZ, NX> &H,
                        const Eigen::Matrix<double, NZ, NZ> &R)
    {
        // innovation covariance
        Eigen::Matrix<double, NZ, NZ> S = H * P * H.transpose() + R;
        // kalman gain
        Eigen::Matrix<double, NX, NZ> K = S.transpose().ldlt().solve(H * P.transpose()).transpose();
        // update state estimate
        x = x + K * (z - h * x);
        // update state covariance estimate
        P = (state_matrix_t::Identity() - K * H) * P;
    }

    void update_sensor_gyro(const sensor_imu_t &imu_sensor)
    {
        Eigen::Matrix<double, 1, 1> z = imu_sensor.tail<1>();
        Eigen::Matrix<double, 1, NX> h;
        h << 0, 0, 0, 0, 0, 1;

        update_generic(z, h, h, R_gyro);
    }

    void update_sensor_mouse(const sensor_mouse_t &mouse_sensor)
    {
        Eigen::Matrix<double, 2, 2> rotation_matrix;
        holohover.world_to_body_rotation_matrix(x, rotation_matrix);

        Eigen::Matrix<double, 2, NX> h;
        h.setZero();
        h.block<2, 2>(0, 2) = rotation_matrix;

        Eigen::Matrix<double, 2, NX> H = h;
        H(0, 4) = -sin(x(4)) * x(2) + cos(x(4)) * x(3);
        H(1, 4) = -cos(x(4)) * x(2) - sin(x(4)) * x(3);

        update_generic(mouse_sensor, h, H, R_mouse);
    }

    void update_sensor_pose(const sensor_pose_t &pose_sensor)
    {
        Eigen::Matrix<double, 3, NX> h;
        h << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 0, 0, 1, 0;

        update_generic(pose_sensor, h, h, R_pose);
    }
};

#endif //HOLOHOVER_GNC_HOLOHOVER_EKF_HPP
