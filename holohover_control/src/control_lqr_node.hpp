#ifndef HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_NODE_HPP

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

class HolohoverControlLQRNode : public rclcpp::Node
{
public:
    HolohoverControlLQRNode();
private:
    HolohoverProps holohover_props;
    ControlLQRSettings control_settings;

    Holohover holohover;

    Holohover::state_t<double> state;
    Holohover::control_force_t<double> motor_velocities;
    Holohover::control_acc_t<double> last_control_acc;
    Holohover::control_force_t<double> last_control_signal;
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
    void ref_callback(const  holohover_msgs::msg::HolohoverState &pose);
};

#endif //HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_NODE_HPP
