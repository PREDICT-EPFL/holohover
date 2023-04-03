#ifndef HOLOHOVER_GNC_HOLOHOVER_CONTROL_EXP_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_CONTROL_EXP_NODE_HPP

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_gnc/riccati_solver.hpp"
#include "holohover_gnc/models/holohover_model.hpp"
#include "holohover_gnc/utils/load_holohover_props.hpp"

#define IDLE_SIGNAL 		0.03 // signal always applied to keep motors moving
#define EVALUATION_DIST_THR 0.03

struct ControlLQRSettings
{
    double period;

    double weight_x;
    double weight_y;
    double weight_v_x;
    double weight_v_y;
    double weight_yaw;
    double weight_w_z;

    double weight_a_x;
    double weight_a_y;
    double weight_w_dot_z;
};

struct ControlExpSettings
{   
    double rand_sig_duration;
    double rand_sig_max;
    double rand_sig_min;
};

ControlLQRSettings load_control_lqr_settings(rclcpp::Node &node)
{
    ControlLQRSettings settings;

    if (node.get_parameter("period", settings.period) &&
        node.get_parameter("weight_x", settings.weight_x) &&
        node.get_parameter("weight_y", settings.weight_y) &&
        node.get_parameter("weight_v_x", settings.weight_v_x) &&
        node.get_parameter("weight_v_y", settings.weight_v_y) &&
        node.get_parameter("weight_yaw", settings.weight_yaw) &&
        node.get_parameter("weight_w_z", settings.weight_w_z) &&
        node.get_parameter("weight_a_x", settings.weight_a_x) &&
        node.get_parameter("weight_a_y", settings.weight_a_y) &&
        node.get_parameter("weight_w_dot_z", settings.weight_w_dot_z)) {}
    else
    {
        RCLCPP_INFO(node.get_logger(), "Failed to load control lqr settings");
    }
    return settings;
}

ControlExpSettings load_control_exp_settings(rclcpp::Node &node)
{
    ControlExpSettings settings2;

    if (node.get_parameter("rand_sig_duration", settings2.rand_sig_duration) &&
        node.get_parameter("rand_sig_max", settings2.rand_sig_max) &&
        node.get_parameter("rand_sig_min", settings2.rand_sig_min)) {}
    else
    {
        RCLCPP_INFO(node.get_logger(), "Failed to load control exp settings");
    }
    return settings2;
}

class HolohoverControlExpNode : public rclcpp::Node
{
public:
    HolohoverControlExpNode();
private:
    HolohoverProps holohover_props;
    ControlLQRSettings control_settings;
    ControlExpSettings exp_settings;

    Holohover holohover;

    Holohover::state_t<double> state;
    geometry_msgs::msg::Pose2D ref;
    Eigen::Matrix<double, Holohover::NA, Holohover::NX> K;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr reference_subscription;

    void init_topics();
    void init_timer();
    void publish_control();
    void state_callback(const holohover_msgs::msg::HolohoverStateStamped &state_msg);
    void ref_callback(const geometry_msgs::msg::Pose2D &pose);
    
    void rand_sig(Holohover::control_force_t<double>& u_signal);
    void evaluation_pos(Holohover::state_t<double>& state_ref, const Holohover::state_t<double>& state);
};

#endif //HOLOHOVER_GNC_HOLOHOVER_CONTROL_EXP_NODE_HPP
