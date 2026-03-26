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
        holohover(holohover_props, 0.1),
        home_pos(std::make_tuple(control_settings.home_x, control_settings.home_y)), // need to make it more general
        goal_pos(std::make_tuple(control_settings.goal_x, control_settings.goal_y))
{
        init_topics();
        control_settings.solver == "ipopt" ? setup_ipopt(control_settings) : setup_hpipm(control_settings);
        init_timer();
}

void HolohoverControlMPCNode::setup_ipopt(ControlMPCSettings control_settings)
{    
    
    double puck_radius = 0.05;
    double hover_radius = 0.07;

    opti = Opti();

    // 1. Decision Variables
    x = opti.variable(nx, N+1);
    u = opti.variable(nu, N);
    MX slack = opti.variable(4, N+1); // [xmin, xmax, ymin, ymax]

    // 2. Parameters
    x0 = opti.parameter(nx);
    puck_state = opti.parameter(nx, 1); // [px, py, pvx, pvy, ...]

    // 3. Basic Constraints 
    opti.subject_to(vec(slack) >= 0); 
    opti.subject_to(x(Slice(), 0) == x0);

    // Motor thrust limits, otherwise hover goes crazy
    opti.subject_to(opti.bounded(0.0, u, control_settings.control_limit));

    // 4. Switching logic for defensive play (if the puck is going away from the hovercraft or is sufficiently far, hovercraft comes home)
    MX puck_x = puck_state(0); 
    MX puck_vx = puck_state(2);

    double dir_sign = (std::get<0>(goal_pos) > std::get<0>(home_pos)) ? 1.0 : -1.0; // -1.0 for h0
    double midline = (std::get<0>(home_pos) + std::get<0>(goal_pos)) / 2.0; // -0.1 default

    MX dist_into_our_side = dir_sign * (midline - puck_x); // positive then on our side
    MX velocity_toward_goal = dir_sign * puck_vx;

    is_away = if_else(dist_into_our_side > 0.0, 0.0, 1.0); 
    MX is_retreating = if_else(velocity_toward_goal > 0.1, 1.0, 0.0);
    is_away = casadi::MX::fmax(is_away, is_retreating); // turned on either on ourside or retreating

    // Hardcorded table boundaries ! (TODO: have to make it more general)
    double x_max = 1.0;
    double y_lim = 0.5;

    MX goal_pos_dx = casadi::MX::vertcat({std::get<0>(goal_pos), std::get<1>(goal_pos)});
    MX home_pos_dx = casadi::MX::vertcat({std::get<0>(home_pos), std::get<1>(home_pos)});

    // ----- Start of the Cost function ------------
    MX obj = 0;

    MX momentum_rewards = 0;
    MX distance_rewards = 0;
    MX p_pos_k = puck_state(Slice(0, 2));
    MX p_vel_k = puck_state(Slice(2, 4));
    double target_yaw = 0.0;

    MX final_strike_spot;
    MX final_unit_dir;

    // 4. Big soup for constraints + objectives ...
    for (int k = 0; k < N; ++k) {
        if (dir_sign < 0) { 
                // Hover 0 (Top side): Must stay in positive X
                opti.subject_to(x(0, k) <= x_max + slack(1, k));
            } else { 
                // Hover 1 (Bottom side): Must stay in negative X
                opti.subject_to(x(0, k) >= -x_max - slack(0, k));
        }
        opti.subject_to(x(1, k) >= -y_lim - slack(2, k));
        opti.subject_to(x(1, k) <=  y_lim + slack(3, k));

        obj += is_away * control_settings.weight_comehome * casadi::MX::sumsqr(x(Slice(0, 2), k) - home_pos_dx);
        obj += is_away * (control_settings.weight_comehome) * casadi::MX::sumsqr(x(Slice(2, 4), k));
        
        Holohover::state_t<MX> xk_mx;
        Holohover::control_force_t<MX> uk_signal_mx; 
        for (int i = 0; i < nx; ++i) xk_mx(i) = x(i, k);
        for (int i = 0; i < nu; ++i) uk_signal_mx(i) = u(i, k);

        Holohover::control_force_t<MX> uk_thrust_newtons;
        holohover.signal_to_thrust<MX>(uk_signal_mx, uk_thrust_newtons);

        Holohover::control_acc_t<MX> u_acc_mx;
        holohover.control_force_to_acceleration<MX>(xk_mx, uk_thrust_newtons, u_acc_mx);

        Holohover::state_t<MX> x_next_mx;
        x_next_mx.setZero();

        // Ad * xk
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                x_next_mx(i) += holohover.Ad(i, j) * xk_mx(j);
            }
        }

        // + Bd * u_acc
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < na; ++j) {
                x_next_mx(i) += holohover.Bd(i, j) * u_acc_mx(j);
            }
        }

        for (int i = 0; i < nx; ++i) {
            opti.subject_to(x(i, k + 1) == x_next_mx(i));
        }

        // Predicting a puck trajectory
        p_pos_k += p_vel_k * control_settings.period;

        // Reflect (y < -0.5)
        MX is_outside_bottom = p_pos_k(1) < -y_lim + puck_radius;
        p_pos_k(1) = if_else(is_outside_bottom, -2.0 * y_lim + 2.0 * puck_radius - p_pos_k(1), p_pos_k(1));
        p_vel_k(1) = if_else(is_outside_bottom, -p_vel_k(1) * 0.8, p_vel_k(1));

        // Reflect (y > 0.5)
        MX is_outside_top = p_pos_k(1) > y_lim - puck_radius;
        p_pos_k(1) = if_else(is_outside_top, 2.0 * y_lim - 2.0 * puck_radius - p_pos_k(1), p_pos_k(1));
        p_vel_k(1) = if_else(is_outside_top, -p_vel_k(1) * 0.8, p_vel_k(1));

        p_vel_k *= 0.993; // Friction

        // Computing where to hit such that it would strike towards a goal at every time step
        vec_to_goal = goal_pos_dx - p_pos_k;
        MX unit_dir = vec_to_goal / (norm_2(vec_to_goal) + 1e-6);
        MX strike_spot = p_pos_k - unit_dir * (puck_radius + hover_radius - 0.02);
        strike_trajectory.push_back(strike_spot);
        
        MX dist_sq_k = sumsqr(x(Slice(0, 2), k) - strike_spot);
        MX dist_lin = sqrt(dist_sq_k + 1e-6);

        // approach_mode is 1.0 when far, strike_mode is 1.0 when close
        // threshold is control_settings.scale_momentum 
        MX approach_mode = 1.0 / (1.0 + exp(- control_settings.weight_y * (dist_lin - control_settings.scale_momentum)));
        MX strike_mode = 1.0 - approach_mode;

        obj += (1.0 - is_away) * approach_mode * control_settings.weight_distance * dist_sq_k;
        
        // strike: Maximize velocity along the unit vector
        MX v_m_proj = dot(x(Slice(2, 4), k), unit_dir);
        momentum_rewards += (1.0 - is_away) * strike_mode * v_m_proj;

        // Regularization to prevent the spinning too much
        obj += control_settings.weight_yaw * sumsqr(x(4, k) - target_yaw);
        obj += control_settings.weight_w_z * sumsqr(x(5, k));

        if (k == N - 1) final_strike_spot = strike_spot;
    }

    // Terminal constraint at final strike spot to smooth out striking motion
    obj += (1.0 - is_away) * control_settings.weight_distance * sumsqr(x(Slice(0,2), N) - final_strike_spot);
    
    MX slack_penalty = 100000.0 * casadi::MX::sumsqr(slack);
    obj += control_settings.weight_motor *sumsqr(u) + slack_penalty- control_settings.weight_momentum * momentum_rewards;
    opti.minimize(obj);
    
    // Solve!
    Dict solver_opts;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = false;
    solver_opts["ipopt.max_iter"] = 100;  // tolerance to small, jit 
    solver_opts["ipopt.tol"] = 1e-6;
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
        msg.state_trajectory[i].v_x = static_cast<double>(x_opt(2,i));
        msg.state_trajectory[i].v_y = static_cast<double>(x_opt(3,i));
        msg.state_trajectory[i].yaw = static_cast<double>(x_opt(4,i));
        msg.state_trajectory[i].w_z = static_cast<double>(x_opt(5,i));
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

    x_ref_vec(0) = ref.x;
    x_ref_vec(1) = ref.y;
    x_ref_vec(2) = ref.v_x;
    x_ref_vec(3) = ref.v_y;
    x_ref_vec(4) = ref.yaw;
    x_ref_vec(5) = ref.w_z;

    for (int i = 0; i < nx; ++i) { 
        x0_val(i) = state(i);
        puck_val(i) = x_ref_vec(i);
    }
    opti.set_value(x0, x0_val);   
    opti.set_value(puck_state, puck_val);   

    // Solve MPC
    auto t_start = std::chrono::steady_clock::now();
    try
    {
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
        
    }
    catch (std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "MPC solve failed: %s", e.what());

        // Publish control message
        holohover_msgs::msg::HolohoverControlStamped control_msg;
        control_msg.header.frame_id = "body";
        control_msg.header.stamp = this->now();

        control_msg.motor_a_1 = static_cast<double>(holohover_props.idle_signal);
        control_msg.motor_a_2 = static_cast<double>(holohover_props.idle_signal);
        control_msg.motor_b_1 = static_cast<double>(holohover_props.idle_signal);
        control_msg.motor_b_2 = static_cast<double>(holohover_props.idle_signal);
        control_msg.motor_c_1 = static_cast<double>(holohover_props.idle_signal);
        control_msg.motor_c_2 = static_cast<double>(holohover_props.idle_signal);

        //control_publisher->publish(control_msg);

        return;
    }

    auto t_end = std::chrono::steady_clock::now();
    long duration_us =
        std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();

    std::cout << "duration_ms = " << duration_us / 1000.0 << std::endl;

    // Extract first optimal control input
    DM u0 = u_opt(Slice(), 0);

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
    state(4) = msg_state.state_msg.yaw;
    state(5) = msg_state.state_msg.w_z;
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

