#ifndef HOLOHOVER_GNC_HOLOHOVER_RIGID_BODY_2D_ACC_DIST_EFK
#define HOLOHOVER_GNC_HOLOHOVER_RIGID_BODY_2D_ACC_DIST_EFK

#include <cmath>
#include <Eigen/Dense>

#include "holohover_common/models/holohover_model.hpp"

namespace holohover
{

class DisturbanceHolohoverEKF
{
public:
    // State is defined as z = (x, y, theta, x_dot, y_dot, theta_dot, dist_x, dist_y, dist_theta, motor_speed * 6)
    using State = Eigen::Matrix<double, 15, 1>;
    // we estimate the covariance only for first 9 states
    using StateCovariance = Eigen::Matrix<double, 9, 9>;

    // Input is the control signal in [0, 1]⁶
    using Input = Eigen::Matrix<double, 6, 1>;

    // The measurement is m = (x, y, theta)
    using Measurement = Eigen::Matrix<double, 3, 1>;
    using MeasurementCovariance = Eigen::Matrix<double, 3, 3>;

private:
    const Holohover holohover;

    StateCovariance Q;
    MeasurementCovariance R;

    Input last_input;
    State state_estimate;
    StateCovariance covariance_estimate;

public:
    DisturbanceHolohoverEKF(const Holohover& holohover, const StateCovariance& Q, const MeasurementCovariance& R,
                            const State& initial_state, const StateCovariance& initial_covariance) :
    holohover(holohover), Q(Q), R(R), last_input(Input::Zero()),
    state_estimate(initial_state), covariance_estimate(initial_covariance) {}

    DisturbanceHolohoverEKF(const Holohover& holohover, const StateCovariance& Q, const MeasurementCovariance& R,
                            const State& initial_state = State::Zero()) :
    DisturbanceHolohoverEKF(holohover, Q, R, initial_state, Q) {}

    const State& getStateEstimate() const { return state_estimate; }
    const StateCovariance& getCovarianceEstimate() const { return covariance_estimate; }

    State predict_state(double dt)
    {
        // RK4 integration of state
        State x_k1 = state_model(state_estimate);
        State x_k2 = state_model(state_estimate + dt / 2.0 * x_k1);
        State x_k3 = state_model(state_estimate + dt / 2.0 * x_k2);
        State x_k4 = state_model(state_estimate + dt * x_k3);
        State next_state = state_estimate + dt / 6.0 * (x_k1 + 2 * x_k2 + 2 * x_k3 + x_k4);

        wrap_angle(next_state(2));
        return next_state;
    }

    void predict(double dt)
    {
        // RK4 integration of state
        State x_k1 = state_model(state_estimate);
        State x_k2 = state_model(state_estimate + dt / 2.0 * x_k1);
        State x_k3 = state_model(state_estimate + dt / 2.0 * x_k2);
        State x_k4 = state_model(state_estimate + dt * x_k3);
        state_estimate = state_estimate + dt / 6.0 * (x_k1 + 2 * x_k2 + 2 * x_k3 + x_k4);

        // RK4 integration of covariance
        StateCovariance cov_k1 = covariance_dot(state_estimate, covariance_estimate);
        StateCovariance cov_k2 = covariance_dot(state_estimate + dt / 2.0 * x_k1, covariance_estimate + dt / 2.0 * cov_k1);
        StateCovariance cov_k3 = covariance_dot(state_estimate + dt / 2.0 * x_k2, covariance_estimate + dt / 2.0 * cov_k2);
        StateCovariance cov_k4 = covariance_dot(state_estimate + dt * x_k3, covariance_estimate + dt * cov_k3);

        covariance_estimate = covariance_estimate + dt / 6.0 * (cov_k1 + 2 * cov_k2 + 2 * cov_k3 + cov_k4);

        wrap_angle(state_estimate(2));
    }

    void update_measurement(const Measurement& measurement)
    {
        // Measurement model assumes positional measurements, i.e., H = [I_3 0_3 0_3]
        Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
        H.topLeftCorner<3, 3>() = Eigen::Matrix<double, 3, 3>::Identity();

        Measurement innovation = measurement - H * state_estimate.head<9>();
        // wrap angle of innovation to get rid of jumps
        wrap_angle(innovation(2));

        MeasurementCovariance S = H * covariance_estimate * H.transpose() + R;
        Eigen::Matrix<double, 9, 3> K = covariance_estimate * H.transpose() * S.inverse();

        state_estimate.head<9>() += K * innovation;
        covariance_estimate = (StateCovariance::Identity() - K * H) * covariance_estimate;

        wrap_angle(state_estimate(2));
    }

    void update_signal(const Input& signal)
    {
        last_input = signal;
    }

private:
    State state_model(const State& x)
    {
        State x_dot = State::Zero();

        x_dot.head<3>() = x.segment<3>(3); // (x, y, theta)

        Holohover::control_force_t<double> signal = x.segment<6>(9); // motor_speed;
        Holohover::control_force_t<double> thrust;
        holohover.signal_to_thrust(signal, thrust);
        Holohover::state_t<double> holohover_state = x.head<6>();
        Holohover::control_acc_t<double> acc;
        holohover.control_force_to_acceleration(holohover_state, thrust, acc);
        x_dot.segment<3>(3) = acc + x.segment<3>(6);

        x_dot.segment<3>(6).array() = 0; // constant disturbance model

        double tau = std::max(holohover.props.motor_tau, 0.001); // ensure tau is not zero
        x_dot.segment<6>(9) = (last_input - x.segment<6>(9)) / tau; // motor_speed

        return x_dot;
    };

    StateCovariance covariance_dot(const State& x, const StateCovariance& cov)
    {
        Eigen::Matrix<double, 9, 9> F = state_jacobian(x);
        return F * cov + cov * F.transpose() + Q;
    }

    Eigen::Matrix<double, 9, 9> state_jacobian(const State& x)
    {
        std::ignore = x;

        Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Zero();
        F(0, 3) = 1;
        F(1, 4) = 1;
        F(2, 5) = 1;
        F(3, 6) = 1;
        F(4, 7) = 1;
        F(5, 8) = 1;
        // we assume constant acceleration input (state independent)
        // it's not totally true, but keeps the calculations simple
        return F;
    }

    template<typename T>
    void wrap_angle(T& angle)
    {
        while (angle <= -M_PI) {
            angle += 2 * M_PI;
        }
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
    }
};

} // namespace holohover

#endif //HOLOHOVER_GNC_HOLOHOVER_RIGID_BODY_2D_ACC_DIST_EFK