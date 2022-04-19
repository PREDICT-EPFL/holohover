#ifndef HOLOHOVER_GNC_HOLOHOVER_MODEL_HPP
#define HOLOHOVER_GNC_HOLOHOVER_MODEL_HPP

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <vector>
#include "solvers/box_admm.hpp"

struct HolohoverProps
{
    // distance between a propeller pairs
    double propeller_pair_gap_distance;
    // radial distance from the center to the midpoint between a propeller pair
    double propeller_pair_radial_distance;
    // angle of first set of propellers
    double phi_offset;
    // mass of the hovercraft
    double mass;
    // inertia in the z-direction
    double inertia;
    // max thrust of a single propeller
    double max_thrust;
    // polynomial coefficients for signal [1000,2000] to thrust [mN] conversation (coeff of the highest order polynomial first)
    std::vector<double> signal_to_thrust_coeffs;
    // polynomial coefficients for thrust [mN] to signal [1000,2000] conversation (coeff of the highest order polynomial first)
    std::vector<double> thrust_to_signal_coeffs;

    // half the angle between two propeller pairs
    double angle_propeller_pair;
    // distance from center to propeller
    double radius_propeller;
};

class Holohover
{
public:
    static const int NX = 6;
    static const int NU = 6;
    static const int NA = 3;

    // x = (x, y, v_x, v_y, yaw, w_z)
    template<typename T>
    using state_t = Eigen::Matrix<T, NX, 1>;

    // u = (F_1, F_2, F_3, F_4, F_5, F_6)
    template<typename T>
    using control_force_t = Eigen::Matrix<T, NU, 1>;

    // u = (a_x, a_y, w_dot_z)
    template<typename T>
    using control_acc_t = Eigen::Matrix<T, NA, 1>;

    // separation angle between the sets of propellers
    const double phi = 2.0 * M_PI / 3.0;

    // holohover properties
    HolohoverProps props;

    // continuous system dynamics for input u = (a_x, a_y, w_dot_z)
    Eigen::Matrix<double, NX, NX> A;
    Eigen::Matrix<double, NX, NA>  B;

    // discretized system dynamics for input u = (a_x, a_y, w_dot_z)
    double dt;
    Eigen::Matrix<double, NX, NX> Ad;
    Eigen::Matrix<double, NX, NA> Bd;

    explicit Holohover(HolohoverProps &_props, double _dt = 0.01) : props(_props), dt(_dt)
    {
        A << 0, 0, 1, 0, 0, 0,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 1,
             0, 0, 0, 0, 0, 0;

        B << 0, 0, 0,
             0, 0, 0,
             1, 0, 0,
             0, 1, 0,
             0, 0, 0,
             0, 0, 1;

        calculate_discretized_system();
    }

    inline void calculate_discretized_system()
    {
        // Raymond DeCarlo: Linear Systems: A State Variable Approach with Numerical Implementation, Prentice Hall, NJ, 1989, p215
        Eigen::Matrix<double, NX + 3, NX + 3> A_B_block;
        A_B_block.setZero();
        A_B_block.template block<NX, NX>(0, 0) = A;
        A_B_block.template block<NX, 3>(0, NX) = B;
        A_B_block *= dt;
        auto A_B_block_exp = A_B_block.exp();
        Ad = A_B_block_exp.template block<NX, NX>(0, 0);
        Bd = A_B_block_exp.template block<NX, 3>(0, NX);
    }

    template<typename T>
    inline void body_to_world_rotation_matrix(const state_t<T> &x, Eigen::Matrix<T, 2, 2> &rotation_matrix) const noexcept
    {
        rotation_matrix << cos(x(4)), -sin(x(4)),
                           sin(x(4)), cos(x(4));
    }

    template<typename T>
    inline void world_to_body_rotation_matrix(const state_t<T> &x, Eigen::Matrix<T, 2, 2> &rotation_matrix) const noexcept
    {
        rotation_matrix << cos(x(4)), sin(x(4)),
                           -sin(x(4)), cos(x(4));
    }

    template<typename T>
    inline void control_force_to_acceleration_mapping(const state_t<T> &x, Eigen::Matrix<T, NA, NU> &map) const noexcept
    {
        Eigen::Matrix<T, 2, NU> force_to_total_force;
        for (int i = 0; i < 3; i++) {
            // x force component of the first propeller in the propeller pair i
            force_to_total_force(0, 2 * i) = -sin(props.phi_offset + phi * i);
            // y force component of the first propeller in the propeller pair i
            force_to_total_force(1, 2 * i) = cos(props.phi_offset + phi * i);
            // x force component of the second propeller in the propeller pair i
            force_to_total_force(0, 2 * i + 1) = sin(props.phi_offset + phi * i);
            // y force component of the second propeller in the propeller pair i
            force_to_total_force(1, 2 * i + 1) = -cos(props.phi_offset + phi * i);
        }

        Eigen::Matrix<T, 1, NU> force_to_moment;
        for (int i = 0; i < 3; i++)
        {
            // position vector of the first propeller in the propeller pair i
            double rx1 = props.radius_propeller * cos(props.phi_offset + phi * i - props.angle_propeller_pair);
            double ry1 = props.radius_propeller * sin(props.phi_offset + phi * i - props.angle_propeller_pair);
            // force vector of the first propeller in the propeller pair i
            double Fx1 = force_to_total_force(0, 2 * i);
            double Fy1 = force_to_total_force(1, 2 * i);
            // moment induced by the first propeller in the propeller pair i
            force_to_moment(0, 2 * i) = rx1 * Fy1 - ry1 * Fx1;

            // position vector of the second propeller in the propeller pair i
            double rx2 = props.radius_propeller * cos(props.phi_offset + phi * i + props.angle_propeller_pair);
            double ry2 = props.radius_propeller * sin(props.phi_offset + phi * i + props.angle_propeller_pair);
            // force vector of the second propeller in the propeller pair i
            double Fx2 = force_to_total_force(0, 2 * i + 1);
            double Fy2 = force_to_total_force(1, 2 * i + 1);
            // moment induced by the second propeller in the propeller pair i
            force_to_moment(0, 2 * i + 1) = rx2 * Fy2 - ry2 * Fx2;
        }

        // rotation from body to world frame
        Eigen::Matrix<T, 2, 2> rotation_matrix;
        body_to_world_rotation_matrix(x, rotation_matrix);

        // x, y acceleration mapping
        map.template topLeftCorner<2, NU>() = 1.0 / props.mass * rotation_matrix * force_to_total_force;

        // angular acceleration around z-axis mapping
        map.template bottomLeftCorner<1, NU>() = 1.0 / props.inertia * force_to_moment;
    }

    template<typename T>
    inline void control_force_to_acceleration(const state_t<T> &x,
                                              const control_force_t<T> &u_force,
                                              control_acc_t<T> &u_acc) const noexcept
    {
        Eigen::Matrix<T, NA, NU> control_force_to_acceleration_map;
        control_force_to_acceleration_mapping(x, control_force_to_acceleration_map);

        u_acc = control_force_to_acceleration_map * u_force;
    }

    template<typename T>
    inline void control_acceleration_to_force(const state_t<T> &x,
                                              const control_acc_t<T> &u_acc,
                                              control_force_t<T> &u_force) const noexcept
    {
        // We solve a QP to find the minimum energy mapping satisfying max thrust constraints
        // Slacks are added to make sure the QP is always feasible
        //
        // min   ||F_i||^2_2 + mu * ||eps||^2_2
        // s.t.  (a_x,a_y,w_dot_z) = A @ (F_1,...,F_6) + eps
        //       0 <= F_i <= F_max
        //
        // translated into standard form
        // min   0.5 * x'Hx + h'x
        // s.t.  Alb <= Ax <= Aub
        //       lb <= x <= ub
        //
        // with x = (F_i, eps)

        Eigen::Matrix<T, NA, NU> control_force_to_acceleration_map;
        control_force_to_acceleration_mapping(x, control_force_to_acceleration_map);

        double mu = 1e6;
        Eigen::Matrix<T, NU + NA, NU + NA> H;
        Eigen::Matrix<T, NU + NA, 1> h;
        Eigen::Matrix<T, NA, NU + NA> A;
        Eigen::Matrix<T, NA, 1> Alb, Aub;
        Eigen::Matrix<T, NU + NA, 1> lb, ub;

        H.setIdentity();
        H.diagonal().template tail<NA>().setConstant(mu);
        h.setZero();
        A.template topLeftCorner<NA, NU>() = control_force_to_acceleration_map;
        A.template topRightCorner<NA, NA>().setIdentity();
        Alb = u_acc;
        Aub = u_acc;
        lb.template head<NU>().setZero();
        lb.template tail<NA>().setConstant(-1e6);
        ub.template head<NU>().setConstant(props.max_thrust);
        ub.template tail<NA>().setConstant(1e6);

        boxADMM<NU + NA, NA, T> solver;
        solver.solve(H, h, A, Alb, Aub, lb, ub);
        u_force = solver.primal_solution().template head<NU>();
    }

    template<typename T>
    inline void state_dynamics(const state_t<T> &x,
                               const control_force_t<T> &u,
                               state_t<T> &x_dot) const noexcept
    {
        control_acc_t<T> u_acc;
        control_force_to_acceleration(x, u, u_acc);

        x_dot(0) = x(2);
        x_dot(1) = x(3);
        x_dot(2) = u_acc(0);
        x_dot(3) = u_acc(1);
        x_dot(4) = x(5);
        x_dot(5) = u_acc(3);
    }

    template<typename T>
    inline void state_dynamics_discrete(const state_t<T> &x,
                                        const control_force_t<T> &u,
                                        state_t<T> &x_next) const noexcept
    {
        control_acc_t<T> u_acc;
        control_force_to_acceleration(x, u, u_acc);

        x_next = Ad * x + Bd * u_acc;
    }

    template<typename T>
    inline void signal_to_thrust(const control_force_t<T> &u_signal, control_force_t<T> &u_thrust) const noexcept
    {
        // map normalized signal [0, 1] to motor signal [1000, 2000]
        control_force_t<T> u_motor_signal;
        u_motor_signal.array() = 1000 + 1000 * u_signal.array();
        control_force_t<T> u_thrust_mN;
        u_thrust_mN.setZero();
        for (const double& coeff: props.signal_to_thrust_coeffs)
        {
            u_thrust_mN.array() *= u_motor_signal.array();
            u_thrust_mN.array() += coeff;
        }
        // fitted thrust is in mN
        u_thrust = 1e-3 * u_thrust_mN;
    }

    template<typename T>
    inline void thrust_to_signal(const control_force_t<T> &u_thrust, control_force_t<T> &u_signal) const noexcept
    {
        // fitted thrust is in mN
        control_force_t<T> u_thrust_mN = 1e3 * u_thrust.array();
        control_force_t<T> u_motor_signal;
        u_motor_signal.setZero();
        for (const double& coeff: props.thrust_to_signal_coeffs)
        {
            u_motor_signal.array() *= u_thrust_mN.array();
            u_motor_signal.array() += coeff;
        }
        // motor signal [1000, 2000] to normalized signal [0, 1]
        u_signal.array() = 0.001 * u_motor_signal.array() - 1;
    }
};

#endif //HOLOHOVER_GNC_HOLOHOVER_MODEL_HPP
