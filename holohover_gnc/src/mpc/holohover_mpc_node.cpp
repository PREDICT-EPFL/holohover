#include <iostream>
#include <iomanip>
#include <chrono>

#include "laopt/laopt.hpp"

#include "holohover_ocp_node.hpp"
#include "laopt/tools/multiple_shooting.hpp"
#include "laopt/solvers/sqp_solver.hpp"
#include "laopt/solvers/piqp_interface.hpp"

using namespace std::chrono;

HolohoverControlMPCNode::HolohoverControlMPCNode() :
        Node("control_mpc", rclcpp::NodeOptions().allow_undeclared_parameters(true)
                                                 .automatically_declare_parameters_from_overrides(true)),
        holohover_props(load_holohover_pros(*this)),
        control_settings(load_control_mpc_settings(*this)),
        holohover(holohover_props, control_settings.period),
        ocp(control_settings, holohover),
        transcription(ocp),
        tape(laopt::generate_tape(transcription, laopt::generate_sparsity(transcription))),
        opt_problem(transcription, tape),
        solver(opt_problem)
{

        ocp.set_tf(2);

        ocp.u_ub << 1, 1, 1, 1, 1, 1;
        //ocp.u_lb << IDLE_SIGNAL, IDLE_SIGNAL, IDLE_SIGNAL, IDLE_SIGNAL, IDLE_SIGNAL, IDLE_SIGNAL;
        ocp.u_lb << 0, 0, 0, 0, 0, 0;
        ref.x = 0;
        ref.y = 0;
        ref.theta = 0;
        ocp.x_ref << ref.x , ref.y, ref.theta, 0, 0, 0;

        ocp.set_x0({0, 0, 0, 0, 0, 0});

        /* Resampling test parameters */
        const double Ts_max = 0.01;
        const double t_test = 0.166;

        solver.settings().verbose = true;
        solver.settings().hessian_approximation = laopt::hessian_approximation_t::EXACT_NO_CONSTRAINTS;
        // solver.settings().max_watchdog_steps = 0;

        init_topics();
        init_timer();

}

void HolohoverControlMPCNode::init_topics()
{
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControlStamped>(
            "drone/control",
            rclcpp::SensorDataQoS());

    HolohoverTrajectory_publisher = this->create_publisher<holohover_msgs::msg::HolohoverTrajectory>(
            "control/HolohoverTrajectory",
            rclcpp::SensorDataQoS());

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverStateStamped>(
            "navigation/state", 10,
            std::bind(&HolohoverControlMPCNode::state_callback, this, std::placeholders::_1));

    reference_subscription = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "control/ref", 10,
            std::bind(&HolohoverControlMPCNode::ref_callback, this, std::placeholders::_1));
}

void HolohoverControlMPCNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(control_settings.period),
            std::bind(&HolohoverControlMPCNode::publish_control, this));
}

void HolohoverControlMPCNode::publish_trajectory( )
{

    holohover_msgs::msg::HolohoverTrajectory msg;
    msg.header.frame_id = "body";
    msg.header.stamp = this->now();

    //StateTrajectory = Eigen::Matrix<Scalar, ControlProblem::NX, N + 1>;
//    trajectory.FutureTrajectory = transcription.get_X_opt()[0][1];

    Eigen::MatrixXd X_opt = transcription.get_X_opt();
    msg.state_trajectory.resize(20);
    for (int i = 0; i < 20; i++)
    {
        msg.state_trajectory[i].x = X_opt(0,i);
        msg.state_trajectory[i].y = X_opt(1,i);
        msg.state_trajectory[i].v_x = X_opt(2,i);
        msg.state_trajectory[i].v_y = X_opt(3,i);
        msg.state_trajectory[i].yaw = X_opt(4,i);
        msg.state_trajectory[i].w_z = X_opt(5,i);
    }

    HolohoverTrajectory_publisher->publish(msg);
}

void HolohoverControlMPCNode::publish_control()
{
    Holohover::state_t<double> state_ref;
    state_ref.setZero();
    state_ref(0) = ref.x;
    state_ref(1) = ref.y;
    state_ref(4) = ref.theta;
    //state_ref = ocp.x_ref;
    ocp.x_ref = state_ref;

    ocp.set_x0(state);

    // LOOP
    const steady_clock::time_point t_start = steady_clock::now();
    solver.solve();
    const steady_clock::time_point t_end = steady_clock::now();
    const long duration_us = duration_cast<microseconds>(t_end - t_start).count();

//     solver.solve(); // Call second time to test repeatability
//     const steady_clock::time_point t_end2 = steady_clock::now();
//     const long duration2_us = duration_cast<microseconds>(t_end2 - t_end).count();

    /* Print out the solution */
    //print_solution(transcription, opt_problem, duration_us, duration2_us);
    //print_sampled_solution(transcription, Ts_max, t_test);

    Holohover::control_force_t<double> u_signal;
    u_signal = transcription.get_u_at(0);

    //std::cout << "STATE =" <<state << std::endl;
    std::cout << "FUTURE STATE =" <<transcription.get_X_opt() << std::endl;
    std::cout << "FUTURE x =" <<transcription.get_X_opt()(0,5) << std::endl;
    //u_signal.setConstant(0.5);
    // clip between 0 and 1
    u_signal = u_signal.cwiseMax(0).cwiseMin(1);

    publish_trajectory();

    holohover_msgs::msg::HolohoverControlStamped control_msg;
    control_msg.header.frame_id = "body";
    control_msg.header.stamp = this->now();
    control_msg.motor_a_1 = u_signal(0);
    control_msg.motor_a_2 = u_signal(1);
    control_msg.motor_b_1 = u_signal(2);
    control_msg.motor_b_2 = u_signal(3);
    control_msg.motor_c_1 = u_signal(4);
    control_msg.motor_c_2 = u_signal(5);
    control_publisher->publish(control_msg);
}


void HolohoverControlMPCNode::state_callback(const holohover_msgs::msg::HolohoverStateStamped &msg_state)
{
    state(0) = msg_state.state_msg.x;
    state(1) = msg_state.state_msg.y;
    state(2) = msg_state.state_msg.v_x;
    state(3) = msg_state.state_msg.v_y;
    state(4) = msg_state.state_msg.yaw;
    state(5) = msg_state.state_msg.w_z;
}

void HolohoverControlMPCNode::ref_callback(const geometry_msgs::msg::Pose2D &pose)
{
    ref = pose;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlMPCNode>());
    rclcpp::shutdown();
    return 0;
}
