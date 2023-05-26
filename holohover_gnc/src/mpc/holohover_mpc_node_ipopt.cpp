#include <iostream>
#include <iomanip>
#include <chrono>

#include "laopt/laopt.hpp"

#include "holohover_ocp_node.hpp"
#include "laopt/tools/multiple_shooting.hpp"
#include "laopt/solvers/sqp_solver.hpp"
#include "laopt/solvers/piqp_interface.hpp"
#define LAOPT_WITH_IPOPT 1;
#define LAOPT_WITH_OSQP 1;

#ifdef LAOPT_WITH_IPOPT
#include "laopt/ipopt_interface/ipopt_wrapper.hpp"
#endif
#ifdef LAOPT_WITH_OSQP
#include "laopt/solvers/osqp_interface.hpp"
#endif

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

        ocp.set_tf(1);

        ocp.u_ub << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
        ocp.u_lb << IDLE_SIGNAL, IDLE_SIGNAL, IDLE_SIGNAL, IDLE_SIGNAL, IDLE_SIGNAL, IDLE_SIGNAL;
        //ocp.u_lb << 0, 0, 0, 0, 0, 0;
        ocp.x_lb << -1, -1, -0.1, -0.1, -3, -1;
        ocp.x_ub << 1, 1, 0.1, 0.1, 3, 1;
        ref.x = 0;
        ref.y = 0;
        ref.yaw = 0;
        ocp.x_ref << ref.x , ref.y, 0, 0, ref.yaw , 0;

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
    //state_ref = ocp.x_ref;
    ocp.x_ref = state_ref;
    std::cout << "CURRENT STATE =" << state << std::endl;

    ocp.set_x0(state);

    // LOOP
    const steady_clock::time_point t_start = steady_clock::now();
    solver.solve();
    const steady_clock::time_point t_end = steady_clock::now();
    const long duration_us = duration_cast<microseconds>(t_end - t_start).count();
    std::cout << "duration_ms  =" <<duration_us/1000 << std::endl;
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
    std::cout << "First OUTPUT =" <<transcription.get_U_opt() << std::endl;
    //u_signal.setConstant(0.5);
    // clip between 0 and 1
    //u_signal = u_signal.cwiseMax(0).cwiseMin(1);

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

#ifdef LAOPT_WITH_IPOPT
        {
            std::cout << "Multiple Shooting - Ipopt\n";

            using Solver = laopt::IpoptWrapper<OptProblem>;
            Solver solver(opt_problem);

            solve_and_print(transcription, opt_problem, solver);
        }
#endif

#ifdef LAOPT_WITH_OSQP
        {
            std::cout << "Multiple Shooting - SQP\n";

            using Solver = laopt::SQPSolver<OptProblem, laopt::OSQPSolver<OptProblem::scalar_t>>;
            Solver solver(opt_problem);
            solver.settings().verbose = true;
            solver.settings().hessian_approximation = laopt::hessian_approximation_t::EXACT_NO_CONSTRAINTS;

            solve_and_print(transcription, opt_problem, solver);
        }
#endif
    }

    /* Solve with Radau Collocation transcription */#include <iostream>
#include <chrono>

#include "laopt/laopt.hpp"

#include "double_integrator_ocp.hpp"
#include "laopt/tools/multiple_shooting.hpp"
#include "laopt/tools/radau_collocation.hpp"
#ifdef LAOPT_WITH_IPOPT
#include "laopt/ipopt_interface/ipopt_wrapper.hpp"
#endif
#include "laopt/solvers/sqp_solver.hpp"
#ifdef LAOPT_WITH_OSQP
#include "laopt/solvers/osqp_interface.hpp"
#endif

#include "examples_helper.hpp"

int main()
{
    using namespace std::chrono;

    /* Choose OCP and Transcription */
    using Ocp = DoubleIntegratorOcp;

    /* Construct OCP and set OCP-specific properties */
    Ocp ocp;

    ocp.set_tf(1.2);

    ocp.u_ub << 10;
    ocp.u_lb << -3;

    ocp.x_ref << 1, 0;

    ocp.set_x0({0.1, 0.2});               // for demonstration, last setting counts
    ocp.x0_lb = ocp.x0_ub = {0.1, 0.2};   // for demonstration, last setting counts
    ocp.x0_ub << 0.1, 0.2;                // for demonstration, last setting counts
    ocp.x0_lb << -0.1, -0.2;

    /* Resampling test parameters */
    const double Ts_max = 0.02;
    const double t_test = 0.166;

    auto solve_and_print = [&](auto& transcription, auto& opt_problem, auto& solver)
    {
        const steady_clock::time_point t_start = steady_clock::now();
        solver.solve();
        const steady_clock::time_point t_end = steady_clock::now();
        const long duration_us = duration_cast<microseconds>(t_end - t_start).count();

        solver.solve(); // Call second time to test repeatability
        const steady_clock::time_point t_end2 = steady_clock::now();
        const long duration2_us = duration_cast<microseconds>(t_end2 - t_end).count();

        /* Print out the solution */
        print_solution(transcription, opt_problem, duration_us, duration2_us);
        print_sampled_solution(transcription, Ts_max, t_test);
    };

    /* Solve with Multiple Shooting transcription */
    if (true)
    {
        const int N = 20;
        using Transcription = laopt_tools::MultipleShooting<Ocp, N>;

        /* Define specific Tape, laOPT, and IPOPT problem types for the resulting NLP */
        using Tape = laopt::TapeInfo<Transcription>;
        using OptProblem = laopt::Problem<Transcription>;

        /* Construct transcription for OCP, optionally generate/store tape for that combination */
        Transcription transcription(ocp);
        Tape tape = laopt::generate_tape(transcription, laopt::generate_sparsity(transcription));

        /* Construct laOPT problem for transcribed OCP using according tape */
        OptProblem opt_problem(transcription, tape); // Tape is optional here and could also be generated internally



    return 0;
}
    if (true)
    {
        const int D_poly = 4;
        const int N_segs = 3;
        using Transcription = laopt_tools::RadauCollocation<Ocp, N_segs, D_poly>;

        /* Define specific Tape and laOPT problem types for the resulting NLP */
        using Tape = laopt::TapeInfo<Transcription>;
        using OptProblem = laopt::Problem<Transcription>;

        /* Construct transcription for OCP, optionally generate/store tape for that combination */
        Transcription transcription(ocp);
        Tape tape = laopt::generate_tape(transcription, laopt::generate_sparsity(transcription));

        /* Construct laOPT problem for transcribed OCP using according tape */
        OptProblem opt_problem(transcription, tape); // Tape is optional here and could also be generated internally

#ifdef LAOPT_WITH_IPOPT
        {
            std::cout << "Radau Collocation - Ipopt\n";

            using Solver = laopt::IpoptWrapper<OptProblem>;
            Solver solver(opt_problem);

            solve_and_print(transcription, opt_problem, solver);
        }
#endif

#ifdef LAOPT_WITH_OSQP
        {
            std::cout << "Radau Collocation - SQP\n";

            using Solver = laopt::SQPSolver<OptProblem, laopt::OSQPSolver<OptProblem::scalar_t>>;
            Solver solver(opt_problem);
            solver.settings().verbose = true;
            solver.settings().hessian_approximation = laopt::hessian_approximation_t::EXACT_NO_CONSTRAINTS;

            solve_and_print(transcription, opt_problem, solver);
        }
#endif
    }



