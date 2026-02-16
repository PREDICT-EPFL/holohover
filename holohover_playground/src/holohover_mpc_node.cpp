#include <iostream>
#include <iomanip>
#include <chrono>

#include "holohover_mpc_node.hpp"
#include "casadi/casadi.hpp"

using namespace std::chrono;
using namespace casadi;

HolohoverControlMPCNode::HolohoverControlMPCNode() :
        Node("control_mpc"),
        holohover_props(load_holohover_pros(declare_parameter<std::string>("holohover_props_file"))),
        control_settings(load_control_mpc_settings(*this)),
        holohover(holohover_props, control_settings.period)
        // ocp(control_settings, holohover),
        // transcription(ocp),
        // tape(laopt::generate_tape(transcription, laopt::generate_sparsity(transcription))),
        // opt_problem(transcription, tape),
        // solver(opt_problem)
{
        // Setting up solver
        opti = Opti(); // optimization problem

        // changes by the solver
        x = opti.variable(nx, N+1);
        u = opti.variable(nu, N); // virtual commands ax, ay, gamma

        // updates everytime
        x0 = opti.parameter(nx);
        x_ref = opti.parameter(nx, N+1);

        // Eigen::MatrixXd Ad_eigen = holohover.Ad;
        // Eigen::MatrixXd Bd_eigen = holohover.Bd;
        // std::cout << "Ad_eigen.rows() = " << Ad_eigen.rows()
        //         << ", Ad_eigen.cols() = " << Ad_eigen.cols() << std::endl;
        // std::cout << "Bd_eigen.rows() = " << Bd_eigen.rows()
        //         << ", Bd_eigen.cols() = " << Bd_eigen.cols() << std::endl;

        // casadi::DM Ad = casadi::DM::zeros(nx, nx);
        // casadi::DM Bd = casadi::DM::zeros(nx, 3);

        // for (int i = 0; i < nx; ++i)
        // for (int j = 0; j < nx; ++j)
        //         Ad(i,j) = Ad_eigen(i,j);

        // for (int i = 0; i < nx; ++i)
        // for (int j = 0; j < 3; ++j)
        //         Bd(i,j) = Bd_eigen(i,j);

        opti.subject_to(x(Slice(), 0) == x0);

        for (int k = 0; k < N; ++k) {
                // 1. Convert CasADi slices to fixed-size Eigen::Matrix<MX, NX, 1>
                Holohover::state_t<MX> xk_mx;
                Holohover::control_force_t<MX> uk_mx;

                for (int i = 0; i < nx; ++i) xk_mx(i) = x(i, k);
                for (int i = 0; i < nu; ++i) uk_mx(i) = u(i, k);

                // 2. Call RK4 discrete nonlinear dynamics
                Holohover::state_t<MX> x_next_mx;
                holohover.non_linear_state_dynamics_discrete<MX>(xk_mx, uk_mx, x_next_mx);

                // 3. Convert x_next back to CasADi MX column vector
                MX x_next_casadi = MX::zeros(nx, 1);
                for (int i = 0; i < nx; ++i) x_next_casadi(i) = x_next_mx(i);

                // 4. Add MPC constraint
                opti.subject_to(x(Slice(), k+1) == x_next_casadi);
        }


        // State and input bounds
        // opti.subject_to(opti.bounded(-0.4, x(0,Slice()), 0.4));
        // opti.subject_to(opti.bounded(-0.4, x(1,Slice()), 0.4));
        // opti.subject_to(opti.bounded(-0.5, x(2,Slice()), 0.5));
        // opti.subject_to(opti.bounded(-0.5, x(3,Slice()), 0.5));
        // opti.subject_to(opti.bounded(-0.5, x(4,Slice()), 0.5));
        // opti.subject_to(opti.bounded(-0.5, x(5,Slice()), 0.5));

        opti.subject_to(opti.bounded(
                holohover_props.idle_signal,
                u,
                0.2));

        // Cost function
        DM Q = DM::eye(nx);
        DM R = 0.01 * DM::eye(nu);

        MX cost = 0;
        
        for (int k = 0; k < N; ++k) {
                MX x_err = x(Slice(),k) - x_ref(Slice(),k);
                cost += mtimes(mtimes(x_err.T(), Q), x_err);
                cost += mtimes(mtimes(u(Slice(),k).T(), R), u(Slice(),k));
        }

        // Terminal cost
        MX x_err_terminal = x(Slice(),N) - x_ref(Slice(),N);
        cost += mtimes(mtimes(x_err_terminal.T(), Q), x_err_terminal);

        opti.minimize(cost);

        // Solver settings
        Dict opts;
        opts["print_time"]      = false;        // Opti-level logging
        opts["ipopt.print_level"] = 0;          // IPOPT solver verbosity (0 = silent)
        opts["ipopt.max_iter"]    = 100;        // maximum iterations
        opts["ipopt.tol"]         = 1e-6;       // convergence tolerance
        opts["ipopt.linear_solver"] = "mumps";  // can also use "ma57" or "ma27" if available
        opts["ipopt.hessian_approximation"] = "exact"; // or "limited-memory" for large problems

        // Set the solver to IPOPT
        opti.solver("ipopt", opts);



        // ocp.set_tf(1);
        // ocp.u_ub.setConstant(0.2);
        // ocp.u_lb.setConstant(holohover_props.idle_signal);
        // //ocp.u_lb << 0, 0, 0, 0, 0, 0;
        // ocp.x_lb << -0.4, -0.4, -0.5, -0.5, -2, -0.5;
        // ocp.x_ub << 0.4, 0.4, 0.5, 0.5, 2, 0.5;
        // ref.x = 0;
        // ref.y = 0;
        // ref.yaw = 0;
        // ocp.x_ref << ref.x , ref.y, 0, 0, ref.yaw , 0;

        // ocp.set_x0({0, 0, 0, 0, 0, 0});

        // /* Resampling test parameters */
        // // const double Ts_max = 0.01;
        // // const double t_test = 0.166;

        // solver.settings().verbose = false;
        // solver.settings().hessian_approximation = laopt::hessian_approximation_t::EXACT_NO_CONSTRAINTS;
        // // solver.settings().max_watchdog_steps = 0;

        init_topics();
        init_timer();
}


void HolohoverControlMPCNode::init_topics()
{
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControlStamped>(
            "control",
            rclcpp::SensorDataQoS());

    // laopt_frequency_publisher = this->create_publisher<holohover_msgs::msg::HolohoverLaoptSpeedStamped>(
    //         "laopt_speed",
    //         rclcpp::SensorDataQoS());

    HolohoverTrajectory_publisher = this->create_publisher<holohover_msgs::msg::HolohoverTrajectory>(
            "HolohoverTrajectory",
            rclcpp::SensorDataQoS());

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverStateStamped>(
            "state", 10,
            std::bind(&HolohoverControlMPCNode::state_callback, this, std::placeholders::_1));

    reference_subscription = this->create_subscription<holohover_msgs::msg::HolohoverState>(
            "state_ref", 10,
            std::bind(&HolohoverControlMPCNode::ref_callback, this, std::placeholders::_1));

    puck_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/puck/pose", 10,
        std::bind(&HolohoverControlMPCNode::puck_pose_callback, this, std::placeholders::_1));
    
}

void HolohoverControlMPCNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(control_settings.period),
            std::bind(&HolohoverControlMPCNode::publish_control, this));
}

void HolohoverControlMPCNode::publish_trajectory()
{
    holohover_msgs::msg::HolohoverTrajectory msg;
    msg.header.frame_id = "body";
    msg.header.stamp = this->now();

    // Extract optimized state trajectory from CasADi

    int N_pred = x_opt.size2();  // number of columns

    msg.state_trajectory.resize(N_pred);

    for (int i = 0; i < N_pred; ++i)
    {
        msg.state_trajectory[i].x   = static_cast<double>(x_opt(0,i));
        msg.state_trajectory[i].y   = static_cast<double>(x_opt(1,i));
        msg.state_trajectory[i].v_x = static_cast<double>(x_opt(2,i));
        msg.state_trajectory[i].v_y = static_cast<double>(x_opt(3,i));
        msg.state_trajectory[i].yaw = static_cast<double>(x_opt(4,i));
        msg.state_trajectory[i].w_z = static_cast<double>(x_opt(5,i));
    }

    HolohoverTrajectory_publisher->publish(msg);
}


void HolohoverControlMPCNode::publish_control()
{
    // Build reference state vector
    casadi::DM x_ref_vec = casadi::DM::zeros(6);

    x_ref_vec(0) = ref.x;
    x_ref_vec(1) = ref.y;
    x_ref_vec(2) = ref.v_x;
    x_ref_vec(3) = ref.v_y;
    x_ref_vec(4) = ref.yaw;
    x_ref_vec(5) = ref.w_z;

    // Build reference trajectory (constant over horizon)
    casadi::DM Xref_mat = casadi::DM::repmat(x_ref_vec, 1, N + 1);

    // Set parameters
    casadi::DM x0_val = casadi::DM::zeros(nx);
    for (int i = 0; i < nx; ++i) x0_val(i) = state(i);
    opti.set_value(x0, x0_val);      
    opti.set_value(x_ref, Xref_mat);

    std::cout << "CURRENT STATE = " << state << std::endl;
    std::cout << "REF STATE = " << x_ref_vec << std::endl;

    // Solve MPC
    auto t_start = std::chrono::steady_clock::now();
    try
    {
        auto sol = opti.solve();

        DM x_sol = sol.value(x);
        DM u_sol = sol.value(u);

        x_opt = x_sol;
        u_opt = u_sol;

        opti.set_initial(x, x_sol);
        opti.set_initial(u, u_sol);

    }
    catch (std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "MPC solve failed: %s", e.what());
        return;
    }

    auto t_end = std::chrono::steady_clock::now();
    long duration_us =
        std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();

    std::cout << "duration_ms = " << duration_us / 1000.0 << std::endl;

    // Extract first optimal control input
    DM u0 = u_opt(Slice(), 0);

    // Saturate control 
    for (int i = 0; i < nu; ++i)
    {
        double val = static_cast<double>(u0(i));
        val = std::max(val, holohover_props.idle_signal);
        val = std::min(val, 1.0);
        u0(i) = val;
    }

    // Publish predicted trajectory
    publish_trajectory();

    // Publish control message
    holohover_msgs::msg::HolohoverControlStamped control_msg;
    control_msg.header.frame_id = "body";
    control_msg.header.stamp = this->now();

    control_msg.motor_a_1 = static_cast<double>(u0(0));
    control_msg.motor_a_2 = static_cast<double>(u0(1));
    control_msg.motor_b_1 = static_cast<double>(u0(2));
    control_msg.motor_b_2 = static_cast<double>(u0(3));
    control_msg.motor_c_1 = static_cast<double>(u0(4));
    control_msg.motor_c_2 = static_cast<double>(u0(5));

    control_publisher->publish(control_msg);

}


// void HolohoverControlMPCNode::publish_laopt_speed(const long &duration_us )
// {
//     speed_msg.header.stamp = this->now();

//     float msg_frequency = duration_us;
//     speed_msg.duration_msg = msg_frequency/1000;
//     laopt_frequency_publisher->publish(speed_msg);
// }


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

void HolohoverControlMPCNode::puck_pose_callback(const geometry_msgs::msg::PoseStamped &puck_pose) 
{
    ref.x = puck_pose.pose.position.x;
    ref.y = puck_pose.pose.position.y;
    ref.yaw = 0;
    ref.v_x = 0;
    ref.v_y = 0;
    ref.w_z = 0;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlMPCNode>());
    rclcpp::shutdown();
    return 0;
}

