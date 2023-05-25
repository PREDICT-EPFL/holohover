#ifndef LAOPT_DOUBLE_INTEGRATOR_OCP_HPP
#define LAOPT_DOUBLE_INTEGRATOR_OCP_HPP

// End user (level 1)

#include <Eigen/Dense>

#include "laopt/laopt.hpp"
#include "laopt/tools/control_problem_base.hpp"
#include "laopt/tools/multiple_shooting.hpp"
#include "laopt/solvers/sqp_solver.hpp"
#include "laopt/solvers/piqp_interface.hpp"

#include "holohover_gnc/models/holohover_model.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_gnc/utils/load_holohover_props.hpp"
#include "holohover_msgs/msg/holohover_trajectory.hpp"


#define IDLE_SIGNAL 0.03 // signal always applied to keep motors moving

struct ControlMPCSettings
{
    double period;

    double weight_x;
    double weight_y;
    double weight_v_x;
    double weight_v_y;
    double weight_yaw;
    double weight_w_z;

    double weight_motor;
};

ControlMPCSettings load_control_mpc_settings(rclcpp::Node &node)
{
    ControlMPCSettings settings;
    if (node.get_parameter("period", settings.period) &&
        node.get_parameter("weight_x", settings.weight_x) &&
        node.get_parameter("weight_y", settings.weight_y) &&
        node.get_parameter("weight_v_x", settings.weight_v_x) &&
        node.get_parameter("weight_v_y", settings.weight_v_y) &&
        node.get_parameter("weight_yaw", settings.weight_yaw) &&
        node.get_parameter("weight_w_z", settings.weight_w_z) &&
        node.get_parameter("weight_motor", settings.weight_motor)) {}
    else
    {
        RCLCPP_INFO(node.get_logger(), "Failed to load control mpc settings");
    }
    return settings;
}

class HolohoverOcp : public laopt_tools::ControlProblemBase</*Scalar*/ double, /*NX*/ 6, /*NU*/ 6>
{
public:
    /* Static parameters */
    // Eigen::Matrix<Scalar, NX, NX> A{{0, 1},
    //                                 {0, 0}};
    // Eigen::Matrix<Scalar, NX, NU> B{{0},
    //                                 {1}};
    State x_ref{0, 0, 0, 0, 0, 0};

    Eigen::Vector<Scalar, NX> Q;
    Eigen::Vector<Scalar, NU> R;

    Holohover holohover;

    HolohoverOcp(ControlMPCSettings &control_settings, Holohover &holohover_) : holohover(holohover_)
    {
        Q << control_settings.weight_x, control_settings.weight_y,
             control_settings.weight_v_x, control_settings.weight_v_y,
             control_settings.weight_yaw, control_settings.weight_w_z;
        R << control_settings.weight_motor, control_settings.weight_motor, control_settings.weight_motor,
             control_settings.weight_motor, control_settings.weight_motor, control_settings.weight_motor;
    
    };

    /* Override function implementations from base class ------------------------------ */
    template<typename T> // T is scalar type
    T lagrange_term_impl(const Eigen::Ref<const state_t<T>> &x,
                         const Eigen::Ref<const input_t<T>> &u,
                         const Eigen::Ref<const param_t<T>> &p)
    {
        return (x_ref - x).dot(Q.asDiagonal() * (x_ref - x)) + u.dot(R.asDiagonal() * u);
    }

    template<typename T, typename Ttf> // T is scalar type
    T mayer_term_impl(const Eigen::Ref<const state_t<T>> &xf,
                      const Eigen::Ref<const param_t<T>> &p,
                      const Ttf &tf)
    {
        return (x_ref - xf).dot(Q.asDiagonal() * (x_ref - xf));
    }

    template<typename T> // T is scalar type
    state_t<T> dynamics_impl(const Eigen::Ref<const state_t<T>> &x,
                             const Eigen::Ref<const input_t<T>> &u,
                             const Eigen::Ref<const param_t<T>> &p)
    {
        holohover_props(load_holohover_pros(*this));
        holohover(HolohoverProps, simulation_settings.period);
        Holohover::control_force_t<double> current_control_force;
        holohover.signal_to_thrust(u, current_control_force);
        Holohover::control_acc_t<double> current_control_acc;
        holohover.control_force_to_acceleration(x, current_control_force, current_control_acc);


        // continuous system dynamics for input u = (a_x, a_y, w_dot_z)
        Eigen::Matrix<double, NX, NX> A;
        Eigen::Matrix<double, NX, 3> B;
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
        
        state_t<T> x_dot = A * x + B * current_control_acc;
        // state_t<T> x_dot;
        // holohover.template nonlinear_state_dynamics<T>(x, u, x_dot);
        return x_dot;
    }

};

class HolohoverControlMPCNode : public rclcpp::Node
{
public:
    static constexpr int N = 25;

    using Ocp = HolohoverOcp;
    using Transcription = laopt_tools::MultipleShooting<Ocp, N>;
    using Tape = laopt::TapeInfo<Transcription>;
    using OptProblem = laopt::Problem<Transcription>;
    using Solver = laopt::SQPSolver<OptProblem, laopt::PIQPSolver<OptProblem::scalar_t>>;
public:
    HolohoverControlMPCNode();
private:
    HolohoverProps holohover_props;
    ControlMPCSettings control_settings;

    Holohover holohover;
    Ocp ocp;
    Transcription transcription;
    Tape tape;
    OptProblem opt_problem;
    Solver solver;

    Holohover::state_t<double> state;
    geometry_msgs::msg::Pose2D ref;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_publisher;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverTrajectory>::SharedPtr HolohoverTrajectory_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr reference_subscription;

    void init_topics();
    void init_timer();
    void publish_control();
    void publish_trajectory();
    void state_callback(const holohover_msgs::msg::HolohoverStateStamped &state_msg);
    void ref_callback(const geometry_msgs::msg::Pose2D &pose);
};


#endif //LAOPT_DOUBLE_INTEGRATOR_OCP_HPP
