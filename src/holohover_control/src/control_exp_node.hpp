#ifndef HOLOHOVER_GNC_HOLOHOVER_CONTROL_EXP_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_CONTROL_EXP_NODE_HPP

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_common/riccati_solver.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "control_lqr_settings.hpp"
#include "control_exp_settings.hpp"

#define EVALUATION_DIST_THR 0.03

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
    holohover_msgs::msg::HolohoverState ref;
    Eigen::Matrix<double, Holohover::NA, Holohover::NX> K;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverState>::SharedPtr reference_subscription;

    void init_topics();
    void init_timer();
    void publish_control();
    void state_callback(const holohover_msgs::msg::HolohoverStateStamped &state_msg);
    void ref_callback(const holohover_msgs::msg::HolohoverState &pose);
    void rand_sig(Holohover::control_force_t<double>& u_signal);
    void evaluation_pos(Holohover::state_t<double>& state_ref, const Holohover::state_t<double>& state);
};

#endif //HOLOHOVER_GNC_HOLOHOVER_CONTROL_EXP_NODE_HPP
