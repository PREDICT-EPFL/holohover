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
#include "holohover_common/utils/load_holohover_props.hpp"

#define NB_SIGNALS 	16
#define NB_MOTORS 	6
#define CYCLE_TIME 	5000000 // in us, signal-on time + signal-off time
#define OFF_TIME 	1000000 // in us, signal-off time
#define IDLE_SIGNAL 0.03 // signal always applied to keep motors moving

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

class HolohoverControlSignalNode : public rclcpp::Node
{
public:
    HolohoverControlSignalNode();
private:
    HolohoverProps holohover_props;
    ControlLQRSettings control_settings;

    Holohover holohover;

    Holohover::state_t<double> state;
    geometry_msgs::msg::Pose2D ref;
    Eigen::Matrix<double, Holohover::NA, Holohover::NX> K;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_publisher;

    void init_topics();
    void init_timer();
    void publish_control();
    void comb2signal(Holohover::control_force_t<double>& u_signal, int comb);
    
};

#endif //HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_NODE_HPP
