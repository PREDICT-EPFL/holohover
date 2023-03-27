#include "control_P_node.hpp"

HolohoverControlPNode::HolohoverControlPNode() :
        Node("control_P", rclcpp::NodeOptions().allow_undeclared_parameters(true)
                                                 .automatically_declare_parameters_from_overrides(true)),
        holohover_props(load_holohover_pros(*this)),
        control_settings(load_control_P_settings(*this)),
        holohover(holohover_props, control_settings.period)
{
    // init state
    state.setZero();

    // init ref
    ref.x = 0;
    ref.y = 0;
    ref.theta = 0;

    K << control_settings.Kp, 0, 0, 0, 0, 0,
              0, control_settings.Kp, 0, 0, 0, 0,
              0, 0, control_settings.Kp, 0, 0, 0,
              0, 0, 0, control_settings.Kp, 0, 0,
              0, 0, 0, 0, control_settings.Kp, 0,
              0, 0, 0, 0, 0, control_settings.Kp;;

    init_topics();
    init_timer();
}

void HolohoverControlPNode::init_topics()
{
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControlStamped>(
            "drone/control",
            rclcpp::SensorDataQoS());

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverStateStamped>(
            "navigation/state", 10,
            std::bind(&HolohoverControlPNode::state_callback, this, std::placeholders::_1));
    reference_subscription = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "control/ref", 10,
            std::bind(&HolohoverControlPNode::ref_callback, this, std::placeholders::_1));
}

void HolohoverControlPNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(control_settings.period),
            std::bind(&HolohoverControlPNode::publish_control, this));
}

void HolohoverControlPNode::publish_control()
{
    Holohover::state_t<double> state_ref;
    state_ref.setZero();
    state_ref(0) = ref.x;
    state_ref(1) = ref.y;
    state_ref(4) = ref.theta;

    Holohover::control_acc_t<double> u_acc = -K * (state - state_ref);   
    Holohover::control_force_t<double> u_force;
    holohover.control_acceleration_to_force(state, u_acc, u_force);    
    Holohover::control_force_t<double> u_signal;
    holohover.thrust_to_signal(u_force, u_signal);

    // clip between 0 and 1
    u_signal = u_signal.cwiseMax(0).cwiseMin(1);

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

void HolohoverControlPNode::state_callback(const holohover_msgs::msg::HolohoverStateStamped &state_msg)
{
    state(0) = state_msg.x;
    state(1) = state_msg.y;
    state(2) = state_msg.v_x;
    state(3) = state_msg.v_y;
    state(4) = state_msg.yaw;
    state(5) = state_msg.w_z;
}

void HolohoverControlPNode::ref_callback(const geometry_msgs::msg::Pose2D &pose)
{
    ref = pose;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlPNode>());
    rclcpp::shutdown();
    return 0;
}
