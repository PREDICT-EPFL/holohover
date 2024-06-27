#include "control_lqr_node.hpp"

HolohoverControlLQRNode::HolohoverControlLQRNode() :
        Node("control_lqr"),
        holohover_props(load_holohover_pros(declare_parameter<std::string>("holohover_props_file"))),
        control_settings(load_control_lqr_settings(*this)),
        holohover(holohover_props, control_settings.period)
{
    // init state
    state.setZero();
    disturbance.setZero();
    motor_velocities.setZero();
    // last_control_acc.setZero();
    last_control_signal.setZero();

    // init ref
    ref.x = control_settings.initial_x;
    ref.y = control_settings.initial_y;
    ref.yaw = control_settings.initial_yaw;

    // calculate LQR gain
    Eigen::Matrix<double, Holohover::NX, Holohover::NX> &Ad = holohover.Ad;
    Eigen::Matrix<double, Holohover::NX, Holohover::NA> &Bd = holohover.Bd;

    Eigen::Matrix<double, Holohover::NX, Holohover::NX> Q;
    Q.setZero();
    Q.diagonal() << control_settings.weight_x, control_settings.weight_y,
                    control_settings.weight_v_x, control_settings.weight_v_y,
                    control_settings.weight_yaw, control_settings.weight_w_z;

    Eigen::Matrix<double, Holohover::NA, Holohover::NA> R;
    R.setZero();
    R.diagonal() << control_settings.weight_a_x, control_settings.weight_a_y, control_settings.weight_w_dot_z;

    Eigen::Matrix<double, Holohover::NX, Holohover::NX> P;
    solve_riccati_iteration_discrete(Ad, Bd, Q, R, P);

    K = (R + Bd.transpose() * P * Bd).ldlt().solve(Bd.transpose() * P * Ad);

    
    //GS BEGIN
    // u_acc_curr.setZero();
    u_acc_bc_curr.setZero();
    u_acc_lqr_curr.setZero();
    std::time_t t = std::time(0);   // get time now
    std::tm* now = std::localtime(&t);

    int my_id = 0;
    file_name_lqr << ament_index_cpp::get_package_prefix("holohover_dmpc") << "/../../log/dmpc_lqr_log_agent" << my_id << "_" << (now->tm_year + 1900) << '_' << (now->tm_mon + 1) << '_' <<  now->tm_mday << "_" << now->tm_hour << "_" << now->tm_min << "_" << now->tm_sec <<".csv";
    log_file_lqr = std::ofstream(file_name_lqr.str());
    if (log_file_lqr.is_open())
    {
        log_file_lqr << "mpc_step, x0_1_, x0_2_, x0_3_, x0_4_, x0_5_, x0_6_, xd_1_, xd_2_, xd_5_, u_1_dmpc_, u_2_dmpc_, u_3_dmpc_, u_1_lqr_, u_2_lqr_, u_3_lqr_, u_1_bc_, u_2_bc_, u_3_bc_, u_1_acc_, u_2_acc_, u_3_acc_\n";
        log_file_lqr.close();
    }
    //GS END

    init_topics();
    init_timer();
}

void HolohoverControlLQRNode::init_topics()
{
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControlStamped>(
            "control",
            rclcpp::SensorDataQoS());

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverStateDisturbanceStamped>(
            "state_disturbance", 10,
            std::bind(&HolohoverControlLQRNode::state_callback, this, std::placeholders::_1));
            
    reference_subscription = this->create_subscription<holohover_msgs::msg::HolohoverState>(
            "state_ref", 10,
            std::bind(&HolohoverControlLQRNode::ref_callback, this, std::placeholders::_1));
}

void HolohoverControlLQRNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(control_settings.period),
            std::bind(&HolohoverControlLQRNode::publish_control, this));
}

void HolohoverControlLQRNode::publish_control()
{
    
    motor_velocities = holohover.Ad_motor * motor_velocities + holohover.Bd_motor * last_control_signal;
    
    Holohover::control_force_t<double> u_force_curr;
    Holohover::control_acc_t<double> u_acc_curr;
    holohover.signal_to_thrust(motor_velocities, u_force_curr);
    holohover.control_force_to_acceleration(state, u_force_curr, u_acc_curr);
    

    Holohover::state_t<double> state_ref;
    state_ref.setZero();
    state_ref(0) = ref.x;
    state_ref(1) = ref.y;
    state_ref(4) = ref.yaw;

    // calculate control for next step
    Holohover::state_t<double> state_next = holohover.Ad * state + holohover.Bd * u_acc_curr;
    Holohover::control_acc_t<double> u_acc_next = -K * (state_next - state_ref) - disturbance;

    // calculate thrust bounds for next step
    Holohover::control_force_t<double> u_force_next_min, u_force_next_max;
    holohover.signal_to_thrust<double>(holohover.Ad_motor * motor_velocities + Holohover::control_force_t<double>::Constant(holohover.Bd_motor * holohover_props.idle_signal), u_force_next_min);
    holohover.signal_to_thrust<double>(holohover.Ad_motor * motor_velocities + Holohover::control_force_t<double>::Constant(holohover.Bd_motor * 1.0), u_force_next_max);

    // calculate next thrust and motor velocities
    Holohover::control_force_t<double> u_force_next;
    holohover.control_acceleration_to_force(state_next, u_acc_next, u_force_next, u_force_next_min, u_force_next_max);    
    Holohover::control_force_t<double> motor_velocities_next;
    holohover.thrust_to_signal(u_force_next, motor_velocities_next);

    // calculate control from future motor velocities
    Holohover::control_force_t<double> u_signal = (motor_velocities_next - holohover.Ad_motor * motor_velocities) / holohover.Bd_motor;

    // clip between 0 and 1
    u_signal = u_signal.cwiseMax(holohover_props.idle_signal).cwiseMin(1);

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


    // //GS
    // Eigen::VectorXd state = state;
    // Eigen::VectorXd u_acc_dmpc_curr_buff_log_ = Eigen::VectorXd::Zero(3);
    // Eigen::VectorXd u_acc_lqr_curr = u_acc_lqr_curr;
    // Eigen::VectorXd u_acc_bc_curr = u_acc_bc_curr;
    // Eigen::VectorXd u_acc_curr = u_acc_curr;



    // log current state and input
    log_file_lqr.open(file_name_lqr.str(),std::ios_base::app);
    if (log_file_lqr.is_open())
    {
        log_file_lqr << 0 << "," << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << "," << state(4) << "," << state(5) << "," << state_ref(0) << "," << state_ref(1) << "," << state_ref(2) << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << u_acc_lqr_curr(0) << "," << u_acc_lqr_curr(1) << "," << u_acc_lqr_curr(2) << "," << u_acc_bc_curr(0) << "," << u_acc_bc_curr(1) << "," << u_acc_bc_curr(2) << "," << u_acc_curr(0) << "," << u_acc_curr(1) << "," << u_acc_curr(2) << "\n";        
    }
    log_file_lqr.close();

    // save control inputs for next iterations   
    last_control_signal = u_signal;
    u_acc_bc_curr = u_acc_next;
    u_acc_lqr_curr = u_acc_next;


}

void HolohoverControlLQRNode::state_callback(const holohover_msgs::msg::HolohoverStateDisturbanceStamped &msg_state)
{
    state(0) = msg_state.state_msg.x;
    state(1) = msg_state.state_msg.y;
    state(2) = msg_state.state_msg.v_x;
    state(3) = msg_state.state_msg.v_y;
    state(4) = msg_state.state_msg.yaw;
    state(5) = msg_state.state_msg.w_z;
    disturbance(0) = msg_state.state_msg.dist_x;
    disturbance(1) = msg_state.state_msg.dist_y;
    disturbance(2) = msg_state.state_msg.dist_yaw;
}

void HolohoverControlLQRNode::ref_callback(const holohover_msgs::msg::HolohoverState &pose)
{
    ref = pose;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlLQRNode>());
    rclcpp::shutdown();
    return 0;
}
