#include <iostream>
#include <iomanip>
#include <chrono>

#include "holohover_mpc_node.hpp"


using namespace std::chrono;

HolohoverControlMPCNode::HolohoverControlMPCNode() :
        Node("control_mpc"),
        holohover_props(load_holohover_pros(declare_parameter<std::string>("holohover_props_file"))),
        control_settings(load_control_mpc_settings(*this)),
        holohover(holohover_props, control_settings.period),
        ocp(control_settings, holohover),
        transcription(ocp),
        tape(laopt::generate_tape(transcription, laopt::generate_sparsity(transcription))),
        opt_problem(transcription, tape),
        solver(opt_problem)
{
        ocp.set_tf(1);

        ocp.u_ub.setConstant(0.2);
        ocp.u_lb.setConstant(holohover_props.idle_signal);
        //ocp.u_lb << 0, 0, 0, 0, 0, 0;
        ocp.x_lb << -0.4, -0.4, -0.5, -0.5, -2, -0.5;
        ocp.x_ub << 0.4, 0.4, 0.5, 0.5, 2, 0.5;
        ref.x = 0;
        ref.y = 0;
        ref.yaw = 0;
        ocp.x_ref << ref.x , ref.y, 0, 0, ref.yaw , 0;

        ocp.set_x0({0, 0, 0, 0, 0, 0});

        /* Resampling test parameters */
        // const double Ts_max = 0.01;
        // const double t_test = 0.166;

        solver.settings().verbose = false;
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

    laopt_frequency_publisher = this->create_publisher<holohover_msgs::msg::HolohoverLaoptSpeedStamped>(
            "control/laopt_speed",
            rclcpp::SensorDataQoS());

    HolohoverTrajectory_publisher = this->create_publisher<holohover_msgs::msg::HolohoverTrajectory>(
            "control/HolohoverTrajectory",
            rclcpp::SensorDataQoS());

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverStateStamped>(
            "navigation/state", 10,
            std::bind(&HolohoverControlMPCNode::state_callback, this, std::placeholders::_1));

    reference_subscription = this->create_subscription<holohover_msgs::msg::HolohoverState>(
            "control/state_ref", 10,
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
    state_ref(4) = ref.yaw;
    state_ref(2) = ref.v_x;
    state_ref(3) = ref.v_y;
    state_ref(5) = ref.w_z;
    //state_ref = ocp.x_ref;
    ocp.x_ref = state_ref;
    //std::cout << "CURRENT STATE =" << state << std::endl;

    ocp.set_x0(state);

    holohover_msgs::msg::HolohoverControlStamped control_msg;
    // LOOP
    const steady_clock::time_point t_start = steady_clock::now();
    solver.settings().verbose = true;
    solver.settings().max_iter = 35;
    solver.solve();
    const steady_clock::time_point t_end = steady_clock::now();
    const long duration_us = duration_cast<microseconds>(t_end - t_start).count();
    publish_laopt_speed(duration_us);
    std::cout << "duration_ms  =" <<duration_us/1000 << std::endl;
//     solver.solve(); // Call second time to test repeatability
//     const steady_clock::time_point t_end2 = steady_clock::now();
//     const long duration2_us = duration_cast<duration_us
    Holohover::control_force_t<double> u_signal;
    u_signal = transcription.get_u_at(0);
    u_signal = u_signal.cwiseMax(holohover_props.idle_signal).cwiseMin(1);

    //std::cout << "STATE =" <<state << std::endl;
    //std::cout << "FUTURE STATE =" <<transcription.get_X_opt() << std::endl;
    //std::cout << "First OUTPUT =" <<transcription.get_U_opt() << std::endl;

    publish_trajectory();

    
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

void HolohoverControlMPCNode::publish_laopt_speed(const long &duration_us )
{
    speed_msg.header.stamp = this->now();

    float msg_frequency = duration_us;
    speed_msg.duration_msg = msg_frequency/1000;
    laopt_frequency_publisher->publish(speed_msg);
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

void HolohoverControlMPCNode::ref_callback(const holohover_msgs::msg::HolohoverState &pose)
{
    ref = pose;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlMPCNode>());
    rclcpp::shutdown();
    return 0;
}

