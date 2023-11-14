#ifndef HOLOHOVER_COMMON_HOLOHOVER_MODEL_HPP
#define HOLOHOVER_COMMON_HOLOHOVER_MODEL_HPP

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "piqp/piqp.hpp"
#include "holohover_common/utils/holohover_props.hpp"

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

    // holohover properties
    HolohoverProps props;
    control_force_t<double> min_thrust;
    control_force_t<double> max_thrust;

    // continuous system dynamics for input u = (a_x, a_y, w_dot_z)
    Eigen::Matrix<double, NX, NX> A;
    Eigen::Matrix<double, NX, NA> B;

    // discretized system dynamics for input u = (a_x, a_y, w_dot_z)
    double dt;
    Eigen::Matrix<double, NX, NX> Ad;
    Eigen::Matrix<double, NX, NA> Bd;

    piqp::DenseSolver<double> solver;
    bool solver_initialized = false;

    explicit Holohover(HolohoverProps &_props, double _dt = 0.01) : props(_props), dt(_dt)
    {
        control_force_t<double> min_signal = control_force_t<double>::Constant(props.idle_signal);
        control_force_t<double> max_signal = control_force_t<double>::Constant(1.0);
        signal_to_thrust(min_signal, min_thrust);
        signal_to_thrust(max_signal, max_thrust);
        // std::cout << "hovercraft min thrust: " << min_thrust.transpose() << std::endl;
        // std::cout << "hovercraft max thrust: " << max_thrust.transpose() << std::endl;

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
    inline void nonlinear_state_dynamics(const state_t<T> &x,
                                         const control_force_t<T> &u,
                                         state_t<T> &x_dot) const noexcept
    {
        Holohover::control_force_t<T> force;

        force(0) = u(0)*(u(0)*(u(0)*props.signal_to_thrust_coeffs_motor_1[0] + props.signal_to_thrust_coeffs_motor_1[1]) + props.signal_to_thrust_coeffs_motor_1[2]);
        force(1) = u(1)*(u(1)*(u(1)*props.signal_to_thrust_coeffs_motor_2[0] + props.signal_to_thrust_coeffs_motor_2[1]) + props.signal_to_thrust_coeffs_motor_2[2]);
        force(2) = u(2)*(u(2)*(u(2)*props.signal_to_thrust_coeffs_motor_3[0] + props.signal_to_thrust_coeffs_motor_3[1]) + props.signal_to_thrust_coeffs_motor_3[2]);
        force(3) = u(3)*(u(3)*(u(3)*props.signal_to_thrust_coeffs_motor_4[0] + props.signal_to_thrust_coeffs_motor_4[1]) + props.signal_to_thrust_coeffs_motor_4[2]);
        force(4) = u(4)*(u(4)*(u(4)*props.signal_to_thrust_coeffs_motor_5[0] + props.signal_to_thrust_coeffs_motor_5[1]) + props.signal_to_thrust_coeffs_motor_5[2]);
        force(5) = u(5)*(u(5)*(u(5)*props.signal_to_thrust_coeffs_motor_6[0] + props.signal_to_thrust_coeffs_motor_6[1]) + props.signal_to_thrust_coeffs_motor_6[2]);

        nonlinear_state_dynamics_force<T>(x, force, x_dot);
    }

    template<typename T>
    inline void nonlinear_state_dynamics_force(const state_t<T> &x,
                                               const control_force_t<T> &u,
                                               state_t<T> &x_dot) const noexcept
    {
        T Fx1 = props.learned_motor_vec_a_1[0]*u(0);
        T Fx2 = props.learned_motor_vec_a_2[0]*u(1);
        T Fx3 = props.learned_motor_vec_b_1[0]*u(2);
        T Fx4 = props.learned_motor_vec_b_2[0]*u(3);
        T Fx5 = props.learned_motor_vec_c_1[0]*u(4);
        T Fx6 = props.learned_motor_vec_c_2[0]*u(5);
        
        T Fy1 = props.learned_motor_vec_a_1[1]*u(0);
        T Fy2 = props.learned_motor_vec_a_2[1]*u(1);
        T Fy3 = props.learned_motor_vec_b_1[1]*u(2);
        T Fy4 = props.learned_motor_vec_b_2[1]*u(3);
        T Fy5 = props.learned_motor_vec_c_1[1]*u(4);
        T Fy6 = props.learned_motor_vec_c_2[1]*u(5); // y dot

        T Fx = cos(x(4))*(Fx1+Fx2+Fx3+Fx4+Fx5+Fx6) - sin(x(4))*(Fy1+Fy2+Fy3+Fy4+Fy5+Fy6);
        T Fy = sin(x(4))*(Fx1+Fx2+Fx3+Fx4+Fx5+Fx6) + cos(x(4))*(Fy1+Fy2+Fy3+Fy4+Fy5+Fy6);

        T Mz1 = -(props.motor_pos_a_1[1]-props.CoM[1])*Fx1 + (props.motor_pos_a_1[0]-props.CoM[0])*Fy1;
        T Mz2 = -(props.motor_pos_a_2[1]-props.CoM[1])*Fx2 + (props.motor_pos_a_2[0]-props.CoM[0])*Fy2;
        T Mz3 = -(props.motor_pos_b_1[1]-props.CoM[1])*Fx3 + (props.motor_pos_b_1[0]-props.CoM[0])*Fy3;
        T Mz4 = -(props.motor_pos_b_2[1]-props.CoM[1])*Fx4 + (props.motor_pos_b_2[0]-props.CoM[0])*Fy4;
        T Mz5 = -(props.motor_pos_c_1[1]-props.CoM[1])*Fx5 + (props.motor_pos_c_1[0]-props.CoM[0])*Fy5;
        T Mz6 = -(props.motor_pos_c_2[1]-props.CoM[1])*Fx6 + (props.motor_pos_c_2[0]-props.CoM[0])*Fy6;

        x_dot(0) = x(2); // x
        x_dot(1) = x(3); // y
        x_dot(2) = Fx/props.mass ; // x dot
        x_dot(3) = Fy/props.mass ;// y dot
        x_dot(4) = x(5); // theta
        x_dot(5) = (Mz1+Mz2+Mz3+Mz4+Mz5+Mz6)/props.inertia; // theta dot

        // linear acceleration correction due to CoM
        x_dot(2) += props.CoM[1] * x_dot(5) + props.CoM[0] * x(5);
        x_dot(3) += -props.CoM[0] * x_dot(5) + props.CoM[1] * x(5);
    }

    template<typename T>
    inline void control_force_to_acceleration_mapping(const state_t<T> &x,
                                                      Eigen::Matrix<T, NA, NU> &map,
                                                      control_acc_t<T> &constant) const noexcept
    {
        Eigen::Matrix<T, 2, NU> force_to_total_force;
        Eigen::Matrix<T, 1, NU> force_to_moment;

        // A motors
        int i = 0;

        // x force component of the first propeller in the propeller pair 1
        force_to_total_force(0, 2 * i) = props.learned_motor_vec_a_1[0];
        // y force component of the first propeller in the propeller pair 1
        force_to_total_force(1, 2 * i) = props.learned_motor_vec_a_1[1];
        // x force component of the second propeller in the propeller pair 1
        force_to_total_force(0, 2 * i + 1) = props.learned_motor_vec_a_2[0];
        // y force component of the second propeller in the propeller pair 1
        force_to_total_force(1, 2 * i + 1) = props.learned_motor_vec_a_2[1];

        // position vector of the first propeller in the propeller pair i if CoM centered                        
        double rx1 = props.motor_pos_a_1[0]-props.CoM[0];
        double ry1 = props.motor_pos_a_1[1]-props.CoM[1];

        // force vector of the first propeller in the propeller pair i
        T Fx1 = force_to_total_force(0, 2 * i);
        T Fy1 = force_to_total_force(1, 2 * i);
        // moment induced by the first propeller in the propeller pair i
        force_to_moment(0, 2 * i) = static_cast<T>(rx1) * Fy1 - static_cast<T>(ry1) * Fx1;

        // Learned position
        double rx2 = props.motor_pos_a_2[0]-props.CoM[0];
        double ry2 = props.motor_pos_a_2[1]-props.CoM[1];

        // force vector of the second propeller in the propeller pair i
        T Fx2 = force_to_total_force(0, 2 * i + 1);
        T Fy2 = force_to_total_force(1, 2 * i + 1);
        // moment induced by the second propeller in the propeller pair i
        force_to_moment(0, 2 * i + 1) = static_cast<T>(rx2) * Fy2 - static_cast<T>(ry2) * Fx2;

        // B motors
        i = 1; 

        // x force component of the first propeller in the propeller pair 1
        force_to_total_force(0, 2 * i) = props.learned_motor_vec_b_1[0];
        // y force component of the first propeller in the propeller pair 1
        force_to_total_force(1, 2 * i) = props.learned_motor_vec_b_1[1];
        // x force component of the second propeller in the propeller pair 1
        force_to_total_force(0, 2 * i + 1) = props.learned_motor_vec_b_2[0];
        // y force component of the second propeller in the propeller pair 1
        force_to_total_force(1, 2 * i + 1) = props.learned_motor_vec_b_2[1];

        // Learned position
        rx1 = props.motor_pos_b_1[0]-props.CoM[0];
        ry1 = props.motor_pos_b_1[1]-props.CoM[1];

        // force vector of the first propeller in the propeller pair i
        Fx1 = force_to_total_force(0, 2 * i);
        Fy1 = force_to_total_force(1, 2 * i);
        // moment induced by the first propeller in the propeller pair i
        force_to_moment(0, 2 * i) = static_cast<T>(rx1) * Fy1 - static_cast<T>(ry1) * Fx1;

        // Learned position
        rx2 = props.motor_pos_b_2[0]-props.CoM[0];
        ry2 = props.motor_pos_b_2[1]-props.CoM[1];

        // force vector of the second propeller in the propeller pair i
        Fx2 = force_to_total_force(0, 2 * i + 1);
        Fy2 = force_to_total_force(1, 2 * i + 1);
        // moment induced by the second propeller in the propeller pair i
        force_to_moment(0, 2 * i + 1) = static_cast<T>(rx2) * Fy2 - static_cast<T>(ry2) * Fx2;

        // C motors
        i = 2;

        // x force component of the first propeller in the propeller pair 1
        force_to_total_force(0, 2 * i) = props.learned_motor_vec_c_1[0];
        // y force component of the first propeller in the propeller pair 1
        force_to_total_force(1, 2 * i) = props.learned_motor_vec_c_1[1];
        // x force component of the second propeller in the propeller pair 1
        force_to_total_force(0, 2 * i + 1) = props.learned_motor_vec_c_2[0];
        // y force component of the second propeller in the propeller pair 1
        force_to_total_force(1, 2 * i + 1) = props.learned_motor_vec_c_2[1];

        // Learned position
        rx1 = props.motor_pos_c_1[0]-props.CoM[0];
        ry1 = props.motor_pos_c_1[1]-props.CoM[1];

        // force vector of the first propeller in the propeller pair i
        Fx1 = force_to_total_force(0, 2 * i);
        Fy1 = force_to_total_force(1, 2 * i);
        // moment induced by the first propeller in the propeller pair i
        force_to_moment(0, 2 * i) = static_cast<T>(rx1) * Fy1 - static_cast<T>(ry1) * Fx1;

        // Learned position
        rx2 = props.motor_pos_c_2[0]-props.CoM[0];
        ry2 = props.motor_pos_c_2[1]-props.CoM[1];

        // force vector of the second propeller in the propeller pair i
        Fx2 = force_to_total_force(0, 2 * i + 1);
        Fy2 = force_to_total_force(1, 2 * i + 1);
        // moment induced by the second propeller in the propeller pair i
        force_to_moment(0, 2 * i + 1) = static_cast<T>(rx2) * Fy2 - static_cast<T>(ry2) * Fx2;

        // rotation from body to world frame
        Eigen::Matrix<T, 2, 2> rotation_matrix;
        body_to_world_rotation_matrix(x, rotation_matrix);

        Eigen::Matrix<T, NA, NU> uncorrected_map;
        // x, y acceleration mapping
        uncorrected_map.template topLeftCorner<2, NU>() = 1.0 / props.mass * rotation_matrix * force_to_total_force;

        // angular acceleration around z-axis mapping
        uncorrected_map.template bottomLeftCorner<1, NU>() = 1.0 / props.inertia * force_to_moment;

        // correction matrix for linear accelerations due to CoM
        Eigen::Matrix<T, NA, NA> CoM_correction;
        CoM_correction << 1.0, 0.0, props.CoM[1],
                          0.0, 1.0, -props.CoM[0],
                          0.0, 0.0, 1.0;

        map = CoM_correction * uncorrected_map;
        constant << props.CoM[0] * x(5) * x(5), props.CoM[1] * x(5) * x(5), 0.0;
    }

    template<typename T>
    inline void control_force_to_acceleration(const state_t<T> &x,
                                              const control_force_t<T> &u_force,
                                              control_acc_t<T> &u_acc) const noexcept
    {
        Eigen::Matrix<T, NA, NU> control_force_to_acceleration_map;
        control_acc_t<T> mapping_constant;
        control_force_to_acceleration_mapping(x, control_force_to_acceleration_map, mapping_constant);

        u_acc = control_force_to_acceleration_map * u_force + mapping_constant;
    }

    template<typename T>
    inline void control_acceleration_to_force(const state_t<T> &x,
                                              const control_acc_t<T> &u_acc,
                                              control_force_t<T> &u_force) noexcept
    {
        // We solve a QP to find the minimum energy mapping satisfying max thrust constraints
        // Slacks are added to make sure the QP is always feasible
        //
        // min   ||F_i||^2_2 + mu * (1 - alpha)^2 + mu * (1 - beta)^2
        // s.t.  (alpha * a_x, alpha * a_y, beta * w_dot_z) = M @ (F_1,...,F_6) + g
        //       0 <= F_i <= F_max
        //
        // translated into standard form
        // min   0.5 * x'Px + c'x
        // s.t.  Ax = b
        //       lb <= x <= ub
        //
        // with x = (F_i, alpha, beta)

        Eigen::Matrix<T, NA, NU> control_force_to_acceleration_map;
        control_acc_t<T> mapping_constant;
        control_force_to_acceleration_mapping(x, control_force_to_acceleration_map, mapping_constant);

        double mu = 1e6;
        Eigen::Matrix<T, NU + 2, NU + 2> P;
        Eigen::Matrix<T, NU + 2, 1> c;
        Eigen::Matrix<T, NA, NU + 2> A;
        Eigen::Matrix<T, NA, 1> b;
        Eigen::Matrix<T, 0, NU + 2> G;
        Eigen::Matrix<T, 0, 1> h;
        Eigen::Matrix<T, NU + 2, 1> lb, ub;

        P.setIdentity();
        P.diagonal().template tail<2>().setConstant(mu);
        c.setZero();
        c.template tail<2>().setConstant(-mu);
        A.template topLeftCorner<NA, NU>() = control_force_to_acceleration_map;
        A.template topRightCorner<NA, 2>() << -u_acc(0), 0.0,
                                              -u_acc(1), 0.0,
                                              0,         -u_acc(2);
        b = -mapping_constant;
        lb.template head<NU>() = min_thrust;
        lb.template tail<2>().setConstant(0.0);
        ub.template head<NU>() = max_thrust;
        ub.template tail<2>().setConstant(1.0);

        if (!solver_initialized) {
            solver.setup(P, c, A, b, G, h, lb, ub);
        } else {
            solver.update(piqp::nullopt, piqp::nullopt, A, b);
        }
        solver.settings().verbose = false;
        solver.solve();
        u_force = solver.result().x.template head<NU>();
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
    inline void non_linear_state_dynamics_discrete(const state_t<T> &x,
                                        const control_force_t<T> &u,
                                        state_t<T> &x_next) const noexcept
    {

        // RK4
        Holohover::state_t<T> k1, k2, k3, k4;
        nonlinear_state_dynamics<T>(x, u, k1);
        nonlinear_state_dynamics<T>(x + dt * k1 / 2, u, k2);
        nonlinear_state_dynamics<T>(x + dt * k2 / 2, u, k3);
        nonlinear_state_dynamics<T>(x + dt * k3, u, k4);

        x_next = x + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
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
        // 
        control_force_t<T> u_motor_signal = u_signal.array();  
        control_force_t<T> u_motor_thrust;
        u_motor_thrust.setZero();

        for (std::size_t i=0; i!=props.signal_to_thrust_coeffs_motor_1.size(); ++i)
        {
            u_motor_thrust.array() *= u_motor_signal.array();
        	Eigen::Matrix<T, NU, 1> coeffs;
            coeffs << static_cast<T>(props.signal_to_thrust_coeffs_motor_1[i]),
                      static_cast<T>(props.signal_to_thrust_coeffs_motor_2[i]),
                      static_cast<T>(props.signal_to_thrust_coeffs_motor_3[i]),
                      static_cast<T>(props.signal_to_thrust_coeffs_motor_4[i]),
                      static_cast<T>(props.signal_to_thrust_coeffs_motor_5[i]),
                      static_cast<T>(props.signal_to_thrust_coeffs_motor_6[i]);
            u_motor_thrust = u_motor_thrust + coeffs;
        }
        
        //for (const double& coeff: props.signal_to_thrust_coeffs_motor_1)
        //{
        //    u_thrust_mN.array() *= u_motor_signal.array();
        //    u_thrust_mN.array() += coeff;
        //}
        
        u_thrust = u_motor_thrust;
    }

    template<typename T>
    inline void thrust_to_signal(const control_force_t<T> &u_thrust, control_force_t<T> &u_signal) const noexcept
    {
        // 
        control_force_t<T> u_motor_thrust = u_thrust.array();
        control_force_t<T> u_motor_signal;
        u_motor_thrust = u_motor_thrust.cwiseMin(max_thrust).cwiseMax(0.0);
        //std::cout << "thrust = " << u_motor_thrust << std::endl;
        u_motor_signal = 0.6*u_motor_thrust; // First linear guess
        
        //double tol = 1e-3; // Tolerance for the root
        int maxiter = 5; // Maximum number of iterations

        auto f = [](auto x, auto a, auto b, auto c, auto d) {
            return x*(x*(a*x+b)+c)-d;
        };
        
        auto fprime = [](auto x, auto a, auto b, auto c) {
            return x*(3*a*x+2*b)+c;
        };

        for (int i=0; i<maxiter; i++) {
            double fx_1 = f(u_motor_signal(0),props.signal_to_thrust_coeffs_motor_1[0],props.signal_to_thrust_coeffs_motor_1[1],props.signal_to_thrust_coeffs_motor_1[2],u_motor_thrust(0));
            double fxprime_1 = fprime(u_motor_signal(0),props.signal_to_thrust_coeffs_motor_1[0],props.signal_to_thrust_coeffs_motor_1[1],props.signal_to_thrust_coeffs_motor_1[2]);
            double fx_2 = f(u_motor_signal(1),props.signal_to_thrust_coeffs_motor_2[0],props.signal_to_thrust_coeffs_motor_2[1],props.signal_to_thrust_coeffs_motor_2[2],u_motor_thrust(1));
            double fxprime_2 = fprime(u_motor_signal(1),props.signal_to_thrust_coeffs_motor_2[0],props.signal_to_thrust_coeffs_motor_2[1],props.signal_to_thrust_coeffs_motor_2[2]);
            double fx_3 = f(u_motor_signal(2),props.signal_to_thrust_coeffs_motor_3[0],props.signal_to_thrust_coeffs_motor_3[1],props.signal_to_thrust_coeffs_motor_3[2],u_motor_thrust(2));
            double fxprime_3 = fprime(u_motor_signal(2),props.signal_to_thrust_coeffs_motor_3[0],props.signal_to_thrust_coeffs_motor_3[1],props.signal_to_thrust_coeffs_motor_3[2]);
            double fx_4 = f(u_motor_signal(3),props.signal_to_thrust_coeffs_motor_4[0],props.signal_to_thrust_coeffs_motor_4[1],props.signal_to_thrust_coeffs_motor_4[2],u_motor_thrust(3));
            double fxprime_4 = fprime(u_motor_signal(3),props.signal_to_thrust_coeffs_motor_4[0],props.signal_to_thrust_coeffs_motor_4[1],props.signal_to_thrust_coeffs_motor_4[2]);
            double fx_5 = f(u_motor_signal(4),props.signal_to_thrust_coeffs_motor_5[0],props.signal_to_thrust_coeffs_motor_5[1],props.signal_to_thrust_coeffs_motor_5[2],u_motor_thrust(4));
            double fxprime_5 = fprime(u_motor_signal(4),props.signal_to_thrust_coeffs_motor_5[0],props.signal_to_thrust_coeffs_motor_5[1],props.signal_to_thrust_coeffs_motor_5[2]);
            double fx_6 = f(u_motor_signal(5),props.signal_to_thrust_coeffs_motor_6[0],props.signal_to_thrust_coeffs_motor_6[1],props.signal_to_thrust_coeffs_motor_6[2],u_motor_thrust(5));
            double fxprime_6 = fprime(u_motor_signal(5),props.signal_to_thrust_coeffs_motor_6[0],props.signal_to_thrust_coeffs_motor_6[1],props.signal_to_thrust_coeffs_motor_6[2]);

            double alpha = 1;
            u_motor_signal(0) = u_motor_signal(0) -alpha * fx_1/fxprime_1; // Update x1
            u_motor_signal(1) = u_motor_signal(1) -alpha * fx_2/fxprime_2; // Update x2
            u_motor_signal(2) = u_motor_signal(2) -alpha * fx_3/fxprime_3; // Update x3
            u_motor_signal(3) = u_motor_signal(3) -alpha * fx_4/fxprime_4; // Update x4
            u_motor_signal(4) = u_motor_signal(4) -alpha * fx_5/fxprime_5; // Update x5
            u_motor_signal(5) = u_motor_signal(5) -alpha * fx_6/fxprime_6; // Update x6
            u_motor_signal = u_motor_signal.cwiseMin(1.0).cwiseMax(0.0);
        }

        u_signal.array() = u_motor_signal.array();
    }



    template<typename T>
    inline void thrust_to_signal_old(const control_force_t<T> &u_thrust, control_force_t<T> &u_signal) const noexcept
    {
        // 
        control_force_t<T> u_motor_thrust = u_thrust.array();
        control_force_t<T> u_motor_signal;
        u_motor_signal.setZero();
        
        for (std::size_t i=0; i!=props.thrust_to_signal_coeffs_motor_1.size(); ++i)
        {
            u_motor_signal.array() *= u_motor_thrust.array();
        	Eigen::Matrix<T, NU, 1> coeffs {props.thrust_to_signal_coeffs_motor_1[i],
        									props.thrust_to_signal_coeffs_motor_2[i],
        									props.thrust_to_signal_coeffs_motor_3[i],
        									props.thrust_to_signal_coeffs_motor_4[i],
        									props.thrust_to_signal_coeffs_motor_5[i],
        									props.thrust_to_signal_coeffs_motor_6[i]};
            u_motor_signal = u_motor_signal + coeffs;
        }
        
        
        u_signal.array() = u_motor_signal.array();
    }
};

#endif //HOLOHOVER_COMMON_HOLOHOVER_MODEL_HPP
