#ifndef HOLOHOVER_GNC_HOLOHOVER_CONTROL_P_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_CONTROL_P_NODE_HPP

#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_gnc/riccati_solver.hpp"
#include "holohover_gnc/models/holohover_model.hpp"
#include "holohover_gnc/utils/load_holohover_props.hpp"

struct ControlPSettings
{
    double period;

    double Kp;

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

ControlPSettings load_control_P_settings(rclcpp::Node &node)
{
    ControlPSettings settings;

    if (node.get_parameter("period", settings.period) &&
        node.get_parameter("Kp", settings.Kp) ) {}
    else
    {
        RCLCPP_INFO(node.get_logger(), "Failed to load control P settings");
    }

    return settings;
}

class HolohoverControlPNode : public rclcpp::Node
{
public:
    HolohoverControlPNode();
private:
    HolohoverProps holohover_props;
    ControlPSettings control_settings;

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
};

#endif //HOLOHOVER_GNC_HOLOHOVER_CONTROL_P_NODE_HPP
