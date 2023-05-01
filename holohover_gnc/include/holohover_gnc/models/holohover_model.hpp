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
    // Learned CoM 
    std::vector<double> CoM;
    // inertia in the z-direction
    double inertia;
    // max thrust of a single propeller
    double max_thrust;
    // polynomial coefficients for signal [0,1] to thrust [N] conversation (coeff of the highest order polynomial first)
    std::vector<double> signal_to_thrust_coeffs_motor1;
    std::vector<double> signal_to_thrust_coeffs_motor2;
    std::vector<double> signal_to_thrust_coeffs_motor3;
    std::vector<double> signal_to_thrust_coeffs_motor4;
    std::vector<double> signal_to_thrust_coeffs_motor5;
    std::vector<double> signal_to_thrust_coeffs_motor6;
    // polynomial coefficients for thrust [N] to signal [0,1] conversation (coeff of the highest order polynomial first)
    std::vector<double> thrust_to_signal_coeffs_motor1;
    std::vector<double> thrust_to_signal_coeffs_motor2;
    std::vector<double> thrust_to_signal_coeffs_motor3;
    std::vector<double> thrust_to_signal_coeffs_motor4;
    std::vector<double> thrust_to_signal_coeffs_motor5;
    std::vector<double> thrust_to_signal_coeffs_motor6;
    // half the angle between two propeller pairs
    double angle_propeller_pair;
    // distance from center to propeller
    double radius_propeller;
    double radius_propeller_a;
    double radius_propeller_b;
    double radius_propeller_c;
    double radius_propeller_a_1;
    double radius_propeller_b_1;
    double radius_propeller_c_1;
    double radius_propeller_a_2;
    double radius_propeller_b_2;
    double radius_propeller_c_2;
    // Position of the motor
    std::vector<double> motor_pos_a_1;
    std::vector<double> motor_pos_a_2;
    std::vector<double> motor_pos_b_1;
    std::vector<double> motor_pos_b_2;
    std::vector<double> motor_pos_c_1;
    std::vector<double> motor_pos_c_2;
    // Learned vectors of the motor
    std::vector<double> learned_motor_vec_a_1;
    std::vector<double> learned_motor_vec_a_2;
    std::vector<double> learned_motor_vec_b_1;
    std::vector<double> learned_motor_vec_b_2;
    std::vector<double> learned_motor_vec_c_1;
    std::vector<double> learned_motor_vec_c_2;
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
    Eigen::Matrix<double, NX, NA> B;

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
        // for (int i = 0; i < 3; i++) {
        //     x force component of the first propeller in the propeller pair i
        //     force_to_total_force(0, 2 * i) = -sin(props.phi_offset + phi * i);
        //     y force component of the first propeller in the propeller pair i
        //     force_to_total_force(1, 2 * i) = cos(props.phi_offset + phi * i);
        //     x force component of the second propeller in the propeller pair i
        //     force_to_total_force(0, 2 * i + 1) = sin(props.phi_offset + phi * i);
        //     y force component of the second propeller in the propeller pair i
        //     force_to_total_force(1, 2 * i + 1) = -cos(props.phi_offset + phi * i);
        //}


        Eigen::Matrix<T, 1, NU> force_to_moment;

        // A
        int i =0 ;

        // x force component of the first propeller in the propeller pair 1
        force_to_total_force(0, 2 * i) = props.learned_motor_vec_a_1[0];
        // y force component of the first propeller in the propeller pair 1
        force_to_total_force(1, 2 * i) = props.learned_motor_vec_a_1[1];
        // x force component of the second propeller in the propeller pair 1
        force_to_total_force(0, 2 * i + 1) = props.learned_motor_vec_a_2[0];
        // y force component of the second propeller in the propeller pair 1
        force_to_total_force(1, 2 * i + 1) = props.learned_motor_vec_a_2[1];

        // position vector of the first propeller in the propeller pair i if CoM centered
        //double rx1 = props.radius_propeller * cos(props.phi_offset + phi * i - props.angle_propeller_pair);
        //double ry1 = props.radius_propeller * sin(props.phi_offset + phi * i - props.angle_propeller_pair);

        // Learned position
        double rx1 = props.motor_pos_a_1[0]-props.CoM[0];
        double ry1 = props.motor_pos_a_1[1]-props.CoM[1];

        // force vector of the first propeller in the propeller pair i
        double Fx1 = force_to_total_force(0, 2 * i);
        double Fy1 = force_to_total_force(1, 2 * i);
        // moment induced by the first propeller in the propeller pair i
        force_to_moment(0, 2 * i) = rx1 * Fy1 - ry1 * Fx1;

        // position vector of the second propeller in the propeller pair i
        // double rx2 = props.radius_propeller * cos(props.phi_offset + phi * i + props.angle_propeller_pair);
        // double ry2 = props.radius_propeller * sin(props.phi_offset + phi * i + props.angle_propeller_pair);

        // Learned position
        double rx2 = props.motor_pos_a_2[0]-props.CoM[0];
        double ry2 = props.motor_pos_a_2[1]-props.CoM[1];

        // force vector of the second propeller in the propeller pair i
        double Fx2 = force_to_total_force(0, 2 * i + 1);
        double Fy2 = force_to_total_force(1, 2 * i + 1);
        // moment induced by the second propeller in the propeller pair i
        force_to_moment(0, 2 * i + 1) = rx2 * Fy2 - ry2 * Fx2;
        // B
        i =1 ; 

        // x force component of the first propeller in the propeller pair 1
        force_to_total_force(0, 2 * i) = props.learned_motor_vec_b_1[0];
        // y force component of the first propeller in the propeller pair 1
        force_to_total_force(1, 2 * i) = props.learned_motor_vec_b_1[1];
        // x force component of the second propeller in the propeller pair 1
        force_to_total_force(0, 2 * i + 1) = props.learned_motor_vec_b_2[0];
        // y force component of the second propeller in the propeller pair 1
        force_to_total_force(1, 2 * i + 1) = props.learned_motor_vec_b_2[1];

        // position vector of the first propeller in the propeller pair i
        // rx1 = props.radius_propeller * cos(props.phi_offset + phi * i - props.angle_propeller_pair);
        // ry1 = props.radius_propeller * sin(props.phi_offset + phi * i - props.angle_propeller_pair);

        // Learned position
        rx1 = props.motor_pos_b_1[0]-props.CoM[0];
        ry1 = props.motor_pos_b_1[1]-props.CoM[1];

        // force vector of the first propeller in the propeller pair i
        Fx1 = force_to_total_force(0, 2 * i);
        Fy1 = force_to_total_force(1, 2 * i);
        // moment induced by the first propeller in the propeller pair i
        force_to_moment(0, 2 * i) = rx1 * Fy1 - ry1 * Fx1;

        // position vector of the second propeller in the propeller pair i
        // rx2 = props.radius_propeller * cos(props.phi_offset + phi * i + props.angle_propeller_pair);
        // ry2 = props.radius_propeller * sin(props.phi_offset + phi * i + props.angle_propeller_pair);

        // Learned position
        rx2 = props.motor_pos_b_2[0]-props.CoM[0];
        ry2 = props.motor_pos_b_2[1]-props.CoM[1];

        // force vector of the second propeller in the propeller pair i
        Fx2 = force_to_total_force(0, 2 * i + 1);
        Fy2 = force_to_total_force(1, 2 * i + 1);
        // moment induced by the second propeller in the propeller pair i
        force_to_moment(0, 2 * i + 1) = rx2 * Fy2 - ry2 * Fx2;
        // C
        i =2 ;

        // x force component of the first propeller in the propeller pair 1
        force_to_total_force(0, 2 * i) = props.learned_motor_vec_c_1[0];
        // y force component of the first propeller in the propeller pair 1
        force_to_total_force(1, 2 * i) = props.learned_motor_vec_c_1[1];
        // x force component of the second propeller in the propeller pair 1
        force_to_total_force(0, 2 * i + 1) = props.learned_motor_vec_c_2[0];
        // y force component of the second propeller in the propeller pair 1
        force_to_total_force(1, 2 * i + 1) = props.learned_motor_vec_c_2[1];

        // position vector of the first propeller in the propeller pair i
        // rx1 = props.radius_propeller * cos(props.phi_offset + phi * i - props.angle_propeller_pair);
        // ry1 = props.radius_propeller * sin(props.phi_offset + phi * i - props.angle_propeller_pair);

        // Learned position
        rx1 = props.motor_pos_c_1[0]-props.CoM[0];
        ry1 = props.motor_pos_c_1[1]-props.CoM[1];

        // force vector of the first propeller in the propeller pair i
        Fx1 = force_to_total_force(0, 2 * i);
        Fy1 = force_to_total_force(1, 2 * i);
        // moment induced by the first propeller in the propeller pair i
        force_to_moment(0, 2 * i) = rx1 * Fy1 - ry1 * Fx1;

        // position vector of the second propeller in the propeller pair i
        // rx2 = props.radius_propeller * cos(props.phi_offset + phi * i + props.angle_propeller_pair);
        // ry2 = props.radius_propeller * sin(props.phi_offset + phi * i + props.angle_propeller_pair);

        // Learned position
        rx2 = props.motor_pos_c_2[0]-props.CoM[0];
        ry2 = props.motor_pos_c_2[1]-props.CoM[1];

        // force vector of the second propeller in the propeller pair i
        Fx2 = force_to_total_force(0, 2 * i + 1);
        Fy2 = force_to_total_force(1, 2 * i + 1);
        // moment induced by the second propeller in the propeller pair i
        force_to_moment(0, 2 * i + 1) = rx2 * Fy2 - ry2 * Fx2;
        

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
        // 
        control_force_t<T> u_motor_signal = u_signal.array();
        control_force_t<T> u_motor_thrust;
        u_motor_thrust.setZero();
        
        for (std::size_t i=0; i!=props.signal_to_thrust_coeffs_motor1.size(); ++i)
        {
            u_motor_thrust.array() *= u_motor_signal.array();
        	Eigen::Matrix<T, NU, 1> coeffs {props.signal_to_thrust_coeffs_motor1[i],
        									props.signal_to_thrust_coeffs_motor2[i],
        									props.signal_to_thrust_coeffs_motor3[i],
        									props.signal_to_thrust_coeffs_motor4[i],
        									props.signal_to_thrust_coeffs_motor5[i],
        									props.signal_to_thrust_coeffs_motor6[i]};
            u_motor_thrust = u_motor_thrust + coeffs;
        }
        
        //for (const double& coeff: props.signal_to_thrust_coeffs_motor1)
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
        u_motor_signal.setConstant(0.5);
        
        //double tol = 1e-3; // Tolerance for the root
        int maxiter = 5; // Maximum number of iterations
        //double x; double a; double b; double c; double d;

        auto f = [](auto x, auto a, auto b, auto c, auto d) {
            return a*pow(x, 3) + b*pow(x, 2) + c*x -d;
        };
        
        auto fprime = [](auto x, auto a, auto b, auto c) {
            return 3*a*pow(x, 2) + 2*b*x + c;
        };
            //u_motor_signal.array() *= u_motor_thrust.array();
        if (u_motor_thrust(0)>props.max_thrust) {
            u_motor_signal(0)=1;
        }
        if (u_motor_thrust(1)>props.max_thrust) {
            u_motor_signal(1)=1;
        }
        if (u_motor_thrust(2)>props.max_thrust) {
            u_motor_signal(2)=1;
        }
        if (u_motor_thrust(3)>props.max_thrust) {
            u_motor_signal(3)=1;
        }
        if (u_motor_thrust(4)>props.max_thrust) {
            u_motor_signal(4)=1;
        }
        if (u_motor_thrust(5)>props.max_thrust) {
            u_motor_signal(5)=1;
        }
        if (u_motor_thrust(6)>props.max_thrust) {
            u_motor_signal(6)=1;
        }

        if (u_motor_thrust(0)<0) {
            u_motor_signal(0)=0;
        }
        if (u_motor_thrust(1)<0) {
            u_motor_signal(1)=0;
        }
        if (u_motor_thrust(2)<0) {
            u_motor_signal(2)=0;
        }
        if (u_motor_thrust(3)<0) {
            u_motor_signal(3)=0;
        }
        if (u_motor_thrust(4)<0) {
            u_motor_signal(4)=0;
        }
        if (u_motor_thrust(5)<0) {
            u_motor_signal(5)=0;
        }
        if (u_motor_thrust(6)<0) {
            u_motor_signal(6)=0;
        }

        for (int i=0; i<maxiter; i++) {
            double fx_1 = f(u_motor_signal(0),props.signal_to_thrust_coeffs_motor1[0],props.signal_to_thrust_coeffs_motor1[1],props.signal_to_thrust_coeffs_motor1[2],u_motor_thrust(0));
            double fxprime_1 = fprime(u_motor_signal(0),props.signal_to_thrust_coeffs_motor1[0],props.signal_to_thrust_coeffs_motor1[1],props.signal_to_thrust_coeffs_motor1[2]);
            double fx_2 = f(u_motor_signal(1),props.signal_to_thrust_coeffs_motor2[0],props.signal_to_thrust_coeffs_motor2[1],props.signal_to_thrust_coeffs_motor2[2],u_motor_thrust(1));
            double fxprime_2 = fprime(u_motor_signal(1),props.signal_to_thrust_coeffs_motor2[0],props.signal_to_thrust_coeffs_motor2[1],props.signal_to_thrust_coeffs_motor2[2]);
            double fx_3 = f(u_motor_signal(2),props.signal_to_thrust_coeffs_motor3[0],props.signal_to_thrust_coeffs_motor3[1],props.signal_to_thrust_coeffs_motor3[2],u_motor_thrust(2));
            double fxprime_3 = fprime(u_motor_signal(2),props.signal_to_thrust_coeffs_motor3[0],props.signal_to_thrust_coeffs_motor3[1],props.signal_to_thrust_coeffs_motor3[2]);
            double fx_4 = f(u_motor_signal(3),props.signal_to_thrust_coeffs_motor4[0],props.signal_to_thrust_coeffs_motor4[1],props.signal_to_thrust_coeffs_motor4[2],u_motor_thrust(3));
            double fxprime_4 = fprime(u_motor_signal(3),props.signal_to_thrust_coeffs_motor4[0],props.signal_to_thrust_coeffs_motor4[1],props.signal_to_thrust_coeffs_motor4[2]);
            double fx_5 = f(u_motor_signal(4),props.signal_to_thrust_coeffs_motor5[0],props.signal_to_thrust_coeffs_motor5[1],props.signal_to_thrust_coeffs_motor5[2],u_motor_thrust(4));
            double fxprime_5 = fprime(u_motor_signal(4),props.signal_to_thrust_coeffs_motor5[0],props.signal_to_thrust_coeffs_motor5[1],props.signal_to_thrust_coeffs_motor5[2]);
            double fx_6 = f(u_motor_signal(5),props.signal_to_thrust_coeffs_motor6[0],props.signal_to_thrust_coeffs_motor6[1],props.signal_to_thrust_coeffs_motor6[2],u_motor_thrust(5));
            double fxprime_6 = fprime(u_motor_signal(5),props.signal_to_thrust_coeffs_motor6[0],props.signal_to_thrust_coeffs_motor6[1],props.signal_to_thrust_coeffs_motor6[2]);

            double alpha = 1;
            u_motor_signal(0) = u_motor_signal(0) -alpha * fx_1/fxprime_1; // Update x1
            u_motor_signal(1) = u_motor_signal(1) -alpha * fx_2/fxprime_2; // Update x2
            u_motor_signal(2) = u_motor_signal(2) -alpha * fx_3/fxprime_3; // Update x3
            u_motor_signal(3) = u_motor_signal(3) -alpha * fx_4/fxprime_4; // Update x4
            u_motor_signal(4) = u_motor_signal(4) -alpha * fx_5/fxprime_5; // Update x5
            u_motor_signal(5) = u_motor_signal(5) -alpha * fx_6/fxprime_6; // Update x6
            u_motor_signal = u_motor_signal.cwiseMin(1.0).cwiseMax(0.0);
            }

                    // if (abs(fx_1/fxprime_1+fx_2/fxprime_2+fx_3/fxprime_3+fx_4/fxprime_4+fx_5/fxprime_5+fx_6/fxprime_6) < tol) { // Check convergence
                    //     u_motor_signal[]
                    //     }
                    //u_motor_signal = u_motor_signal + coeffs;
        u_signal.array() = u_motor_signal.array();
    }



    template<typename T>
    inline void thrust_to_signal_old(const control_force_t<T> &u_thrust, control_force_t<T> &u_signal) const noexcept
    {
        // 
        control_force_t<T> u_motor_thrust = u_thrust.array();
        control_force_t<T> u_motor_signal;
        u_motor_signal.setZero();
        
        for (std::size_t i=0; i!=props.thrust_to_signal_coeffs_motor1.size(); ++i)
        {
            u_motor_signal.array() *= u_motor_thrust.array();
        	Eigen::Matrix<T, NU, 1> coeffs {props.thrust_to_signal_coeffs_motor1[i],
        									props.thrust_to_signal_coeffs_motor2[i],
        									props.thrust_to_signal_coeffs_motor3[i],
        									props.thrust_to_signal_coeffs_motor4[i],
        									props.thrust_to_signal_coeffs_motor5[i],
        									props.thrust_to_signal_coeffs_motor6[i]};
            u_motor_signal = u_motor_signal + coeffs;
        }
        
        //for (const double& coeff: props.thrust_to_signal_coeffs_motor1)
        //{
        //    u_motor_signal.array() *= u_thrust_mN.array();
        //    u_motor_signal.array() += coeff;
        //    std::cout << coeff << ", ";
        //}
        
        u_signal.array() = u_motor_signal.array();
    }
};

#endif //HOLOHOVER_GNC_HOLOHOVER_MODEL_HPP
