#include "control_lqr_node.hpp"

HolohoverControlLQRNode::HolohoverControlLQRNode() :
        Node("control_lqr"),
        holohover_props(load_holohover_pros(*this)),
        control_settings(load_control_lqr_settings(*this)),
        holohover(holohover_props, control_settings.period)
{
    // init state
    state.setZero();

    // init ref
    ref.x = 0;
    ref.y = 0;
    ref.yaw = 0;

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

    init_topics();
    init_timer();
}

void HolohoverControlLQRNode::init_topics()
{
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControlStamped>(
            "drone/control",
            rclcpp::SensorDataQoS());

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverStateStamped>(
            "navigation/state", 10,
            std::bind(&HolohoverControlLQRNode::state_callback, this, std::placeholders::_1));
            
    reference_subscription = this->create_subscription<holohover_msgs::msg::HolohoverState>(
            "control/state_ref", 10,
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
    Holohover::state_t<double> state_ref;
    state_ref.setZero();
    state_ref(0) = ref.x;
    state_ref(1) = ref.y;
    state_ref(4) = ref.yaw;

    Holohover::control_acc_t<double> u_acc = -K * (state - state_ref);   
    Holohover::control_force_t<double> u_force;
    holohover.control_acceleration_to_force(state, u_acc, u_force);    
    Holohover::control_force_t<double> u_signal;
    holohover.thrust_to_signal(u_force, u_signal);

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
}

void HolohoverControlLQRNode::state_callback(const holohover_msgs::msg::HolohoverStateStamped &msg_state)
{
    state(0) = msg_state.state_msg.x;
    state(1) = msg_state.state_msg.y;
    state(2) = msg_state.state_msg.v_x;
    state(3) = msg_state.state_msg.v_y;
    state(4) = msg_state.state_msg.yaw;
    state(5) = msg_state.state_msg.w_z;
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
