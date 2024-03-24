#ifndef HOLOHOVER_GNC_HOLOHOVER_RIGID_BODY_2D_EFK
#define HOLOHOVER_GNC_HOLOHOVER_RIGID_BODY_2D_EFK

#include <math.h>
#include <tuple>
#include <Eigen/Dense>

namespace holohover
{

class RigidBody2DEKF
{
public:
    // State is defined as z = (x, y, theta, x_dot, y_dot, theta_dot, x_dot_dot, y_dot_dot, theta_dot_dot)
    using State = Eigen::Matrix<double, 9, 1>;
    using StateCovariance = Eigen::Matrix<double, 9, 9>;
    
    // The measurement is m = (x, y, theta)
    using Measurement = Eigen::Matrix<double, 3, 1>;
    using MeasurementCovariance = Eigen::Matrix<double, 3, 3>;

private:
    StateCovariance Q;
    MeasurementCovariance R;

    State state_estimate;
    StateCovariance covariance_estimate;

public:
    RigidBody2DEKF(const StateCovariance& Q, const MeasurementCovariance& R,
                   const State& initial_state, const StateCovariance& initial_covariance) :
        Q(Q), R(R), state_estimate(initial_state), covariance_estimate(initial_covariance) {}

    RigidBody2DEKF(const StateCovariance& Q, const MeasurementCovariance& R,
                   const State& initial_state = State::Zero()) :
        RigidBody2DEKF(Q, R, initial_state, Q) {}

    const State& getStateEstimate() const { return state_estimate; }
    const StateCovariance& getCovarianceEstimate() const { return covariance_estimate; }

    State predict_state(double timestep)
    {
        // RK4 integration of state
        State x_k1 = state_model(state_estimate);
        State x_k2 = state_model(state_estimate + timestep / 2.0 * x_k1);
        State x_k3 = state_model(state_estimate + timestep / 2.0 * x_k2);
        State x_k4 = state_model(state_estimate + timestep * x_k3);
        State next_state = state_estimate + timestep / 6.0 * (x_k1 + 2 * x_k2 + 2 * x_k3 + x_k4);

        wrap_angle(next_state(2));
        return next_state;
    }

    void predict(double timestep)
    {
        // RK4 integration of state
        State x_k1 = state_model(state_estimate);
        State x_k2 = state_model(state_estimate + timestep / 2.0 * x_k1);
        State x_k3 = state_model(state_estimate + timestep / 2.0 * x_k2);
        State x_k4 = state_model(state_estimate + timestep * x_k3);
        state_estimate = state_estimate + timestep / 6.0 * (x_k1 + 2 * x_k2 + 2 * x_k3 + x_k4);

        // RK4 integration of covariance
        StateCovariance cov_k1 = covariance_dot(state_estimate, covariance_estimate);
        StateCovariance cov_k2 = covariance_dot(state_estimate + timestep / 2.0 * x_k1, covariance_estimate + timestep / 2.0 * cov_k1);
        StateCovariance cov_k3 = covariance_dot(state_estimate + timestep / 2.0 * x_k2, covariance_estimate + timestep / 2.0 * cov_k2);
        StateCovariance cov_k4 = covariance_dot(state_estimate + timestep * x_k3, covariance_estimate + timestep * cov_k3);

        covariance_estimate = covariance_estimate + timestep / 6.0 * (cov_k1 + 2 * cov_k2 + 2 * cov_k3 + cov_k4);

        wrap_angle(state_estimate(2));
    }

    void update(const Measurement& measurement)
    {
        // Mesurement model assumes positional measurements, i.e., H = [I_3 0_3 0_3]
        Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
        H.topLeftCorner<3, 3>() = Eigen::Matrix<double, 3, 3>::Identity();

        Measurement innovation = measurement - H * state_estimate;
        // wrap angle of innovation to get rid of jumps
        wrap_angle(innovation(2));

        MeasurementCovariance S = H * covariance_estimate * H.transpose() + R;
        Eigen::Matrix<double, 9, 3> K = covariance_estimate * H.transpose() * S.inverse();

        state_estimate += K * innovation;
        covariance_estimate = (StateCovariance::Identity() - K * H) * covariance_estimate;

        wrap_angle(state_estimate(2));
    }

private:
    State state_model(const State& x)
    {
        State x_dot = State::Zero();
        x_dot.head<6>() = x.tail<6>();
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

#endif //HOLOHOVER_GNC_HOLOHOVER_RIGID_BODY_2D_EFK