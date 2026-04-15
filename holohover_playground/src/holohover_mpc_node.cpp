#include <iostream>
#include <iomanip>
#include <chrono>
#include <tuple>

#include "holohover_mpc_node.hpp"
#include "casadi/casadi.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


using namespace std::chrono;
using namespace casadi;

HolohoverControlMPCNode::HolohoverControlMPCNode() :
        Node("control_mpc"),
        holohover_props(load_holohover_pros(declare_parameter<std::string>("holohover_props_file"))),
        control_settings(load_control_mpc_settings(*this)),
        holohover(holohover_props, delta_t),
        home_pos(std::make_tuple(control_settings.home_x, control_settings.home_y)), // need to make it more general
        goal_pos(std::make_tuple(control_settings.goal_x, control_settings.goal_y))
{
        init_topics();
        control_settings.solver == "ipopt" ? setup_ipopt(control_settings) : setup_ipopt_old(control_settings);
        init_timer();
}

void HolohoverControlMPCNode::setup_ipopt(ControlMPCSettings control_settings)
{    

    // 0. Physical Parameters
    double puck_radius = 0.05;
    double hover_radius = 0.07;

    double max_vel = 2.0; // of hovercraft (just for solvers)
    double x_lim = 0.9; double y_lim = 0.5; // table

    opti = Opti();

    // 1. Variables
    x = opti.variable(nx, N+1); // [x, y, vx, vy]
    u = opti.variable(nu, N); // [ax, ay]
    MX slack = opti.variable(2, N+1); // [x_min, x_max] 

    
    // 2. Solver Parameters
    x0 = opti.parameter(nx);
    puck_state = opti.parameter(nx, 1); 

    // 3. Constraints
    opti.subject_to(x(Slice(), 0) == x0);
    opti.subject_to(opti.bounded(-control_settings.control_limit, u, control_settings.control_limit)); // to be handled by the low level later
    opti.subject_to(vec(slack) >= 0);

    // 4. Defensive vs Offensive Play Set up
    MX puck_x = puck_state(0); 
    MX puck_vx = puck_state(2);
    MX goal_pos_dx = casadi::MX::vertcat({std::get<0>(goal_pos), std::get<1>(goal_pos)});
    MX home_pos_dx = casadi::MX::vertcat({std::get<0>(home_pos), std::get<1>(home_pos)});
    
    double dir_sign = (std::get<0>(goal_pos) > std::get<0>(home_pos)) ? 1.0 : -1.0; 
    double midline_x = (std::get<0>(home_pos) + std::get<0>(goal_pos)) / 2.0;
    
    MX dist_from_mid = dir_sign * (midline_x - puck_x);
    MX vel_toward_goal = dir_sign * puck_vx;

    double alpha = 60.0;   // steepness of switching condition at the mid point (VERY STEEP)
    double v_offset = 0.2;  // at least this puck velocity to be considered retreating

    MX is_on_opposite_side =  0.5 * (1.0 + casadi::MX::tanh(alpha * -(dist_from_mid)));
    MX is_retreating = 0.5 * (1.0 + casadi::MX::tanh(alpha * (vel_toward_goal - v_offset)));
    is_away = casadi::MX::fmax(is_on_opposite_side, is_retreating);

    // 5. Cost and Dynamics Loop
    MX obj = 0;
    momentum_rewards = 0;
    distance_cost = 0;

    MX p_pos_k = puck_state(Slice(0, 2));
    MX p_vel_k = puck_state(Slice(2, 4));

    // What would be the position that hover needs to be in NOW to hit the puck? - later becomes terminal condition
    // Not very obvious why I am doing this but I believe this guides the momentum reward
    MX robot_pos_now = x(Slice(0, 2), 0);
    MX dist_to_puck_now = casadi::MX::norm_2(robot_pos_now - p_pos_k);
    //MX close_alpha = 0.5 * (1.0 + casadi::MX::tanh(10.0 * (0.4 - dist_to_puck_now)));
    MX close_alpha = casadi::MX::exp(-dist_to_puck_now/0.2);
    MX vec_imm = goal_pos_dx - p_pos_k;
    MX unit_dir_imm = vec_imm / (casadi::MX::norm_2(vec_imm) + 1e-6);
    MX strike_imm = p_pos_k - unit_dir_imm * (puck_radius);

    //double dt = control_settings.period;
    double dt = delta_t;
    for (int k = 0; k < N; ++k) {
        // A. Position update: [x,y]next = [x,y] + [vx,vy]*dt
        opti.subject_to(x(Slice(0, 2), k+1) == x(Slice(0, 2), k) + x(Slice(2, 4), k) * dt);

        // B. Velocity update: [vx,vy]next = [vx,vy] + [ax,ay]*dt
        opti.subject_to(x(Slice(2, 4), k+1) == x(Slice(2, 4), k) + u(Slice(0, 2), k) * dt);
        opti.subject_to(pow(x(2, k), 2) + pow(x(3, k), 2)<= max_vel*max_vel);
        
        // C. Coming home logic
        obj += is_away * control_settings.weight_comehome * casadi::MX::sumsqr(x(Slice(0, 2), k) - home_pos_dx);

        // D. Puck pose prediction with wall reflections 
        p_pos_k += p_vel_k * dt;
        double y_min = -y_lim + puck_radius;
        MX is_outside_left = p_pos_k(1) < y_min;

        p_pos_k(1) = if_else(is_outside_left, 2.0 * y_min - p_pos_k(1), p_pos_k(1));
        p_vel_k(1) = if_else(is_outside_left, -p_vel_k(1) * 0.8, p_vel_k(1));

        double y_max = y_lim - puck_radius;
        MX is_outside_right = p_pos_k(1) > y_max;

        p_pos_k(1) = if_else(is_outside_right, 2.0 * y_max - p_pos_k(1), p_pos_k(1));
        p_vel_k(1) = if_else(is_outside_right, -p_vel_k(1) * 0.8, p_vel_k(1));

        p_vel_k *= 0.993; // Friction

        // E. Strike pose prediction at k - based on the puck pose
        MX vec_to_goal_k = goal_pos_dx - p_pos_k;
        unit_dir_k = vec_to_goal_k / (casadi::MX::norm_2(vec_to_goal_k) + 1e-6);
        MX strike_spot_k = p_pos_k - unit_dir_k * (puck_radius);
        strike_trajectory.push_back(strike_spot_k);

        // F. Positional weight generation for tracking based on the distance from the puck 
        MX dist_sq_imm = casadi::MX::sumsqr(x(Slice(0, 2), k) - strike_imm);
        MX dist_sq_k = casadi::MX::sumsqr(x(Slice(0, 2), k) - strike_spot_k);
        MX dist_lin = casadi::MX::sqrt(dist_sq_k + 1e-6);
        MX vel_sq_k = casadi::MX::sumsqr(x(Slice(2, 4), k));

        // if the puck is close, go to the strike position closest to the puck
        // if the puck is far, go to the strike position further from the puck (be there in advanced) 
        // distance_cost += (close_alpha * dist_sq_imm + (1.0 - close_alpha) * dist_sq_k);
        distance_cost += (1.0 - close_alpha) * dist_sq_k;
        // G. Positional weight generation for momentum maximisation based on the distance from the puck 
        double alpha = control_settings.scale_distance*control_settings.scale_distance; 
        double delta = control_settings.scale_momentum*control_settings.scale_momentum; 
        
        // if the puck is close, go on strike mode (maximises momentum in the direction of the goal)
        // apply this momentum reward only above certain velocity (you NEED to STRIKE)
        MX strike_mode = 0.5 * (1.0 + casadi::MX::tanh(-10.0 * (dist_lin - delta)));
        MX velocity_gate = 0.5 * (1.0 + casadi::MX::tanh(10.0 * (vel_sq_k - 0.1)));
        
        MX v_m_proj = casadi::MX::dot(x(Slice(2, 4), k), unit_dir_k);
        MX v_p_proj = casadi::MX::dot(p_vel_k, unit_dir_k);
        momentum_rewards += strike_mode * velocity_gate * (v_m_proj - v_p_proj);
        // momentum_rewards += strike_mode * (v_m_proj - v_p_proj);
        // H. Boundary constraints
        opti.subject_to(x(1, k) >= -y_lim );
        opti.subject_to(x(1, k) <=  y_lim );
        opti.subject_to(x(0, k) >=  -x_lim-slack(1,k) );
        opti.subject_to(x(0, k) <=  x_lim+slack(0,k) );
    }    

    // Final objective summing
    MX slack_penalty = 1e5 * sumsqr(slack);
    MX control_effort = control_settings.weight_motor * casadi::MX::sumsqr(u);

    obj += control_settings.weight_distance * distance_cost;
    obj += control_settings.weight_yaw * casadi::MX::sumsqr(x(Slice(0, 2), N) - strike_imm); // terminal
    obj += (-control_settings.weight_momentum * momentum_rewards);
    obj += control_effort;
    obj += slack_penalty;
    opti.minimize(obj);
    
    // Solver Settings
    Dict solver_opts;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = false;
    solver_opts["ipopt.max_iter"] = 40; 
    solver_opts["ipopt.tol"] = 1e-4;
    solver_opts["jit"] = true;
    solver_opts["jit_options.flags"] = {"-O3"};
    solver_opts["jit_options.verbose"] = false;
    solver_opts["compiler"] = "shell";         
    opti.solver("ipopt", solver_opts);
}

// This is same as python

void HolohoverControlMPCNode::setup_ipopt_old(ControlMPCSettings control_settings)
{    

    // 0. Physical Parameters
    double puck_radius = 0.05;
    double hover_radius = 0.07;

    double max_vel = 1.5; // of hovercraft (just for solvers)
    double x_max = 1.0; double y_lim = 0.5; // table

    opti = Opti();

    // 1. Variables
    x = opti.variable(nx, N+1); // [x, y, vx, vy]
    u = opti.variable(nu, N); // [ax, ay]
    
    // 2. Solver Parameters
    x0 = opti.parameter(nx);
    puck_state = opti.parameter(nx, 1); 

    // 3. Constraints
    opti.subject_to(x(Slice(), 0) == x0);
    opti.subject_to(opti.bounded(-control_settings.control_limit, u, control_settings.control_limit)); // to be handled by the low level later

    // 4. Defensive vs Offensive Play Set up
    MX puck_x = puck_state(0); 
    MX puck_vx = puck_state(2);
    MX goal_pos_dx = casadi::MX::vertcat({std::get<0>(goal_pos), std::get<1>(goal_pos)});
    MX home_pos_dx = casadi::MX::vertcat({std::get<0>(home_pos), std::get<1>(home_pos)});
    
    double dir_sign = (std::get<0>(goal_pos) > std::get<0>(home_pos)) ? 1.0 : -1.0; 
    double midline_x = (std::get<0>(home_pos) + std::get<0>(goal_pos)) / 2.0;
    
    MX dist_from_mid = dir_sign * (midline_x - puck_x);
    MX vel_toward_goal = dir_sign * puck_vx;

    double alpha = 20.0;   
    double v_offset = 0.2; // at least this velocity

    MX is_on_opposite_side =  0.5 * (1.0 + casadi::MX::tanh(alpha * -(dist_from_mid)));
    MX is_retreating = 0.5 * (1.0 + casadi::MX::tanh(alpha * (vel_toward_goal - v_offset)));
    is_away = casadi::MX::fmax(is_on_opposite_side, is_retreating);

    // 5. Cost and Dynamics Loop
    MX obj = 0;
    momentum_rewards = 0;
    distance_cost = 0;

    MX p_pos_k = puck_state(Slice(0, 2));
    MX p_vel_k = puck_state(Slice(2, 4));
    double dt = 1.5 / N;
    
    MX final_strike_spot;

    for (int k = 0; k < N; ++k) {
        // --- Dynamics (Simple Double Integrator) ---
        MX x_k = x(Slice(), k);
        MX u_k = u(Slice(), k);
        MX x_next = x_k + casadi::MX::vertcat({x_k(Slice(2, 4)), u_k}) * dt;
        opti.subject_to(x(Slice(), k + 1) == x_next);
        opti.subject_to(pow(x(2, k), 2) + pow(x(3, k), 2)<= max_vel*max_vel);
        
        // Defense cost
        obj += is_away * control_settings.weight_comehome * casadi::MX::sumsqr(x(Slice(0, 2), k) - home_pos_dx);

        // --- Puck Trajectory Prediction ---
        p_pos_k += p_vel_k * dt;
        double y_min = -y_lim + puck_radius;
        MX is_outside_left = p_pos_k(1) < y_min;

        p_pos_k(1) = if_else(is_outside_left, 2.0 * y_min - p_pos_k(1), p_pos_k(1));
        p_vel_k(1) = if_else(is_outside_left, -p_vel_k(1) * 0.8, p_vel_k(1));

        double y_max = y_lim - puck_radius;
        MX is_outside_right = p_pos_k(1) > y_max;

        p_pos_k(1) = if_else(is_outside_right, 2.0 * y_max - p_pos_k(1), p_pos_k(1));
        p_vel_k(1) = if_else(is_outside_right, -p_vel_k(1) * 0.8, p_vel_k(1));

        p_vel_k *= 0.993; // Friction

        // --- Strike Geometry ---
        MX vec_to_goal_k = goal_pos_dx - p_pos_k;
        unit_dir_k = vec_to_goal_k / (casadi::MX::norm_2(vec_to_goal_k) + 1e-6);
        MX strike_spot_k = p_pos_k - unit_dir_k * (puck_radius);
        strike_trajectory.push_back(strike_spot_k);

        MX dist_sq_k = casadi::MX::sumsqr(x(Slice(0, 2), k) - strike_spot_k);

        double alpha = control_settings.scale_distance*control_settings.scale_distance; 
        double delta = control_settings.scale_momentum*control_settings.scale_momentum; 
        MX strike_mode = 0.5 * (1.0 + casadi::MX::tanh(20.0 * (delta - dist_sq_k)));

        // MX weight_dist = MX::exp(-(dist_sq_k)/alpha);
        // MX weight_momentum = MX::exp(-(dist_sq_k)/delta);
        MX v_m_proj = casadi::MX::dot(x(Slice(2, 4), k), unit_dir_k);
        MX v_p_proj = casadi::MX::dot(p_vel_k, unit_dir_k);
        momentum_rewards += strike_mode * (v_m_proj - v_p_proj);
        distance_cost += (strike_mode)*(dist_sq_k);
        // Boundary logic
        opti.subject_to(x(1, k) >= -y_lim );
        opti.subject_to(x(1, k) <=  y_lim );
        opti.subject_to(x(0, k) >= -x_max );
        opti.subject_to(x(0, k) <=  x_max );
    }    
    // Final objective summing
    MX control_effort = control_settings.weight_motor * casadi::MX::sumsqr(u);
    obj += control_settings.weight_distance * distance_cost;
    obj += (- control_settings.weight_momentum * momentum_rewards);
    obj += control_effort;
    opti.minimize(obj);
    
    // Solver Settings
    Dict solver_opts;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = false;
    solver_opts["ipopt.max_iter"] = 20; 
    solver_opts["ipopt.tol"] = 1e-4;
    opti.solver("ipopt", solver_opts);
}

void HolohoverControlMPCNode::setup_hpipm(ControlMPCSettings control_settings) 
{        
    // Current not working !
    opti = Opti(); 

    std::vector<MX> x_list;
    std::vector<MX> u_list;
    
    // creating an interleave structure of [x0, u0, x1, u1] ... as per instruction
    for (int k = 0; k < N; ++k) {
        // Stage k
        x_list.push_back(opti.variable(nx)); // Columns for x_k
        u_list.push_back(opti.variable(nu)); // Columns for u_k
    }
    // Terminal Stage
    x_list.push_back(opti.variable(nx));    // Columns for x_N

    x = horzcat(x_list); // apparently it turns the vector in the horizontal slicing
    u = horzcat(u_list);

    x0 = opti.parameter(nx);
    x_ref = opti.parameter(nx, N+1);

    for (int k = 0; k < N; ++k)
    {
        // Stage slices 
        MX xk = x(Slice(), k);
        MX uk = u(Slice(), k);

        Holohover::state_t<MX> x_in;
        Holohover::control_force_t<MX> u_in;
        // need to convert MX to compatible types of the model
        for (int i = 0; i < nx; ++i)
            x_in(i) = xk(i);

        for (int i = 0; i < nu; ++i)
            u_in(i) = uk(i);

        Holohover::state_t<MX> x_out;
        holohover.non_linear_state_dynamics_discrete<MX>(x_in, u_in, x_out);

        std::vector<MX> x_vec(nx);
        for (int i = 0; i < nx; ++i)
            x_vec[i] = x_out(i);

        MX x_next = MX::vertcat(x_vec);
        opti.subject_to(x_next == x(Slice(), k+1));
        if (k == 0) {
            opti.subject_to(x(Slice(), 0) == x0);
        }
    }

    MX cost = 0;

    DM Q = DM::diag(DM({
        control_settings.weight_x,
        control_settings.weight_y,
        control_settings.weight_v_x,
        control_settings.weight_v_y,
        control_settings.weight_yaw,
        control_settings.weight_w_z
    }));

    DM R = DM::diag(DM({
        control_settings.weight_motor,
        control_settings.weight_motor,
        control_settings.weight_motor,
        control_settings.weight_motor,
        control_settings.weight_motor,
        control_settings.weight_motor
    }));

    // stage cost
    for (int k = 0; k < N; ++k) {
        MX err_x = x(Slice(), k) - x_ref(Slice(), k);
        MX uk = u(Slice(), k);
        
        cost += mtimes(err_x.T(), mtimes(Q, err_x));
        cost += mtimes(uk.T(), mtimes(R, uk));
    }

    // terminal cost
    MX err_N = x(Slice(), N) - x_ref(Slice(), N);
    cost += mtimes(err_N.T(), mtimes(Q, err_N));

    opti.minimize(cost);

    //opti.subject_to(x(Slice(), 0) == x0);

    std::vector<int> nx_vec(N + 1, nx);
    std::vector<int> nu_vec(N + 1, nu);
    std::vector<int> ng_vec(N + 1, 0); 

    nu_vec[N] = 0; 
    ng_vec[0] = 6; 
    ng_vec[N]= 0;

    Dict qp_opts;
    qp_opts["N"]  = N;
    qp_opts["nx"] = nx_vec;
    qp_opts["nu"] = nu_vec;
    qp_opts["ng"] = ng_vec;

    Dict hpipm_opts;
    hpipm_opts["mode"] = "speed";
    qp_opts["hpipm"] = hpipm_opts;

    Dict opts;
    opts["qpsol"] = "hpipm";
    opts["expand"] = true;
    opts["qpsol_options"] = qp_opts;

    opti.solver("sqpmethod", opts);

    // // Print the order of variables in the decision vector
    std::cout << "--- Decision Variable Order ---" << std::endl;
    MX z = opti.x();
    std::cout << "Total variables: " << z.size1() << std::endl;
    // This shows the symbolic names (e.g., 'var0', 'var1') and their dimensions
    std::cout << z << std::endl; // decision variable
    // // 1. Grab the Jacobian of the constraints (g) with respect to variables (x)
    // MX g = opti.g();
    // MX J = MX::jacobian(g, z);
    // MX p = opti.p();

    // // 2. Create the Function
    // casadi::Function f_J = casadi::Function("f_J", {z, p}, {J});

    // // 3. Prepare inputs as a vector of DM
    // std::vector<DM> arg = {
    //     DM::zeros(z.nnz(), 1), 
    //     DM::zeros(p.nnz(), 1)
    // };

    // // 4. Call the function and get result
    // std::vector<DM> res = f_J(arg);
    // DM J_numeric = res[0];

    // // 5. Print the full matrix
    // std::cout << "--- Full Jacobian Matrix (N=2) ---" << std::endl;
    // // Use .full() or densify if the matrix is large, but for N=2, this works:
    // std::cout << J_numeric << std::endl;

    // Debugging the same up
    casadi::MX g = opti.g(); // vector of constraints defined by subject_to
    casadi::MX J = casadi::MX::jacobian(g, z);
    casadi::Function f_J = casadi::Function("f_J", {z, opti.p()}, {J});
    std::vector<casadi::DM> J_out = f_J(std::vector<casadi::DM>{
        casadi::DM::zeros(z.nnz(), 1), 
        casadi::DM::zeros(opti.p().nnz(), 1)
    });

    std::cout << "\n--- Jacobian Sparsity Pattern (Spy) ---" << std::endl;
    J_out[0].sparsity().spy(std::cout);
    std::cout << "---------------------------------------\n" << std::endl;
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

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "holohover_trajectory_marker",
        rclcpp::QoS(10).reliable());

    trajectory_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mpc/strike_target", 10);
        
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
        // msg.state_trajectory[i].v_x = static_cast<double>(x_opt(2,i));
        // msg.state_trajectory[i].v_y = static_cast<double>(x_opt(3,i));
        // msg.state_trajectory[i].yaw = static_cast<double>(x_opt(4,i));
        // msg.state_trajectory[i].w_z = static_cast<double>(x_opt(5,i));
    }

    HolohoverTrajectory_publisher->publish(msg);

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "world"; 
    marker.header.stamp = this->now();

    marker.ns = "holohover_traj";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Line width
    marker.scale.x = 0.01;  

    // Color (RGBA)
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    // Lifetime (0 = forever until overwritten)
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);

    for (int i = 0; i < N_pred; ++i)
    {
        geometry_msgs::msg::Point p;
        p.x = static_cast<double>(x_opt(0,i));
        p.y = static_cast<double>(x_opt(1,i));
        p.z = 0.0;  // 2D trajectory

        marker.points.push_back(p);
    }

    marker_pub_->publish(marker);

}

void HolohoverControlMPCNode::publish_dual_trajectories(const casadi::DM& x_lti, const casadi::DM& x_rk4) {
    // Comparison for non-linear vs linear dynamics but I believe two arguments are swtched ^ (#TODO)
    auto create_marker = [this](const casadi::DM& traj, std::string ns, int id, float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.scale.x = 0.02; // Thickness
        marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0;

        for (int i = 0; i < traj.size2(); ++i) {
            geometry_msgs::msg::Point p;
            p.x = static_cast<double>(traj(0, i));
            p.y = static_cast<double>(traj(1, i));
            marker.points.push_back(p);
        }
        return marker;
    };

    marker_pub_->publish(create_marker(x_lti, "lti_prediction", 0, 1.0, 0.0, 0.0)); // RED
    marker_pub_->publish(create_marker(x_rk4, "rk4_prediction", 1, 0.0, 1.0, 0.0)); // GREEN
}

void HolohoverControlMPCNode::publish_control()
{
    if (!state_ready || !puck_ready) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for data...");
        return;
    }
    // Build reference state vector
    casadi::DM x_ref_vec = casadi::DM::zeros(nx);
    casadi::DM puck_val = casadi::DM::zeros(nx);
    casadi::DM x0_val = casadi::DM::zeros(nx);

    // puck state 
    x_ref_vec(0) = ref.x + ref.v_x * delay_seconds;
    x_ref_vec(1) = ref.y + ref.v_y * delay_seconds;
    x_ref_vec(2) = ref.v_x;
    x_ref_vec(3) = ref.v_y;
    // x_ref_vec(4) = ref.yaw;
    // x_ref_vec(5) = ref.w_z;

    for (int i = 0; i < nx; ++i) { 
        x0_val(i) = state(i);
        puck_val(i) = x_ref_vec(i);
    }

    double dt = control_settings.period;

    for (const auto& past_u : control_history) {
        x0_val(0) = x0_val(0) + x0_val(2) * dt + 0.5 * past_u(0) * dt * dt; // x
        x0_val(1) = x0_val(1) + x0_val(3) * dt + 0.5 * past_u(1) * dt * dt; // y

        x0_val(2) = x0_val(2) + past_u(0) * dt; // v_x
        x0_val(3) = x0_val(3) + past_u(1) * dt; // v_y
    }

    opti.set_value(x0, x0_val);   
    opti.set_value(puck_state, puck_val);   

    // Solve MPC + Use suboptimal solution
    auto t_start = std::chrono::steady_clock::now();
    try
    {   
        // use smith predictor to compensate for the timedelay
        auto sol = opti.solve();

        DM x_sol = sol.value(x);
        DM u_sol = sol.value(u);

        x_opt = x_sol;
        u_opt = u_sol;

        casadi::DM u_opt_val = sol.value(u);
        casadi::DM x_start_val = sol.value(x(Slice(), 0));

        for(int k = 0; k < N - 1; ++k) {
            opti.set_initial(x(Slice(0, nx), k), x_sol(Slice(0, nx), k + 1));
            opti.set_initial(u(Slice(0, nu), k), u_sol(Slice(0, nu), k + 1));
        }

        std::vector<casadi::DM> st;
                for(size_t i = 0; i < strike_trajectory.size(); ++i) {
                    casadi::DM spot_val = sol.value(strike_trajectory[i]);
                    st.push_back(spot_val);
                }

        publish_strike_trajectory(st);
        double dist_val = static_cast<double>(sol.value(distance_cost));
        double mom_val  = static_cast<double>(sol.value(momentum_rewards));
        double away_val = static_cast<double>(sol.value(is_away));
        MX home_pos_dx = casadi::MX::vertcat({std::get<0>(home_pos), std::get<1>(home_pos)});


        // Calculate the actual weighted contribution to the objective
        double weighted_dist = control_settings.weight_distance * dist_val;
        double weighted_mom  = control_settings.weight_momentum * mom_val;
        double weighted_home = away_val * control_settings.weight_comehome * static_cast<double>(sol.value(casadi::MX::sumsqr(x(Slice(0, 2), 0) - home_pos_dx)));

        RCLCPP_INFO(this->get_logger(), 
            "--- MPC Balance ---\n"
            "State: %s | Away Signal: %.2f\n"
            "Weighted Dist Cost: %.4f\n"
            "Weighted Mom Reward: %.4f\n"
            "Weighted Home Cost: %.4f",
            (away_val > 0.5 ? "DEFENSE" : "ATTACK"), 
            away_val, weighted_dist, weighted_mom, weighted_home);
        
    }
    catch (std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "MPC solve failed: %s", e.what());
        auto u_suboptimal = opti.debug().value(u);
        auto x_suboptimal = opti.debug().value(x);
        publish_trajectory();
        // Publish control message
        holohover_msgs::msg::HolohoverControlStamped control_msg;
        control_msg.header.frame_id = "body";
        control_msg.header.stamp = this->now();

        control_msg.motor_a_1 = static_cast<double>(u_suboptimal(0,0));
        control_msg.motor_a_2 = static_cast<double>(u_suboptimal(1,0));
        //control_msg.motor_b_1 = static_cast<double>(u_suboptimal(2,0));
        for(int k = 0; k < N - 1; ++k) {
            opti.set_initial(x(Slice(0, nx), k), x_suboptimal(Slice(0, nx), k + 1));
            opti.set_initial(u(Slice(0, nu), k), u_suboptimal(Slice(0, nu), k + 1));
        }
        control_publisher->publish(control_msg);

        return;
    }

    auto t_end = std::chrono::steady_clock::now();
    long duration_us =
        std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();

    std::cout << "duration_ms = " << duration_us / 1000.0 << std::endl;

    // Extract first optimal control input
    DM u0 = u_opt(Slice(), 0);
    control_history.push_back(u0);
    if (control_history.size() > delay_steps) {
        control_history.pop_front();
    }
    

    // Publish predicted trajectory
    publish_trajectory();

    // Publish control message
    holohover_msgs::msg::HolohoverControlStamped control_msg;
    control_msg.header.frame_id = "body";
    control_msg.header.stamp = this->now();

    control_msg.motor_a_1 = static_cast<double>(u0(0));
    control_msg.motor_a_2 = static_cast<double>(u0(1));
    // control_msg.motor_b_1 = static_cast<double>(u0(2));
    // control_msg.motor_b_2 = static_cast<double>(u0(3));
    // control_msg.motor_c_1 = static_cast<double>(u0(4));
    // control_msg.motor_c_2 = static_cast<double>(u0(5));

    control_publisher->publish(control_msg);

}

void HolohoverControlMPCNode::publish_strike_trajectory(const std::vector<casadi::DM>& trajectory) {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t k = 0; k < trajectory.size(); ++k) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "strike_points";
        marker.id = static_cast<int>(k); // Unique ID for each point
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Extract coordinates from DM
        marker.pose.position.x = static_cast<double>(trajectory[k](0));
        marker.pose.position.y = static_cast<double>(trajectory[k](1));
        marker.pose.position.z = 0.05;

        // Size and Color
        marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05;
        marker.color.r = 1.0f - (float)k / trajectory.size(); // Gradient fade
        marker.color.g = (float)k / trajectory.size();
        marker.color.b = 0.5f;
        marker.color.a = 0.8f;

        marker_array.markers.push_back(marker);
    }
    
    // Publish the array
    trajectory_pub_->publish(marker_array);
}

void HolohoverControlMPCNode::state_callback(const holohover_msgs::msg::HolohoverStateStamped &msg_state)
{
    state(0) = msg_state.state_msg.x;
    state(1) = msg_state.state_msg.y;
    state(2) = msg_state.state_msg.v_x;
    state(3) = msg_state.state_msg.v_y;
    // state(4) = msg_state.state_msg.yaw;
    // state(5) = msg_state.state_msg.w_z;
    if (!state_ready) state_ready = true;
    
}

void HolohoverControlMPCNode::ref_callback(const holohover_msgs::msg::HolohoverState &pose)
{
    //ref = pose;
}

void HolohoverControlMPCNode::puck_pose_callback(const geometry_msgs::msg::PoseStamped &puck_pose) 
{
    rclcpp::Time current_time = puck_pose.header.stamp;
    
    if (first_callback) {
        last_position = puck_pose.pose.position;
        last_time = current_time;
        first_callback = false;
        return;
    }

    double dt = (current_time - last_time).seconds();
    
    if (dt > 1e-6) {
        ref.v_x = (puck_pose.pose.position.x - last_position.x) / dt;
        ref.v_y = (puck_pose.pose.position.y - last_position.y) / dt;
        ref.x = puck_pose.pose.position.x;
        ref.y = puck_pose.pose.position.y;
        ref.yaw = 0;
        ref.w_z = 0;

        if (!puck_ready) puck_ready = true;
    }
    last_position = puck_pose.pose.position;
    last_time = current_time;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlMPCNode>());
    rclcpp::shutdown();
    return 0;
}

