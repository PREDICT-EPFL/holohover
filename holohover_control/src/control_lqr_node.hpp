#ifndef HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_NODE_HPP

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "holohover_msgs/msg/holohover_state.hpp"
#include "holohover_msgs/msg/holohover_state_disturbance_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_common/riccati_solver.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "control_lqr_settings.hpp"

//GS
#include <iostream>
#include <fstream>
#include <chrono>
#include <ament_index_cpp/get_package_prefix.hpp>


class HolohoverControlLQRNode : public rclcpp::Node
{
public:
    HolohoverControlLQRNode();
private:
    HolohoverProps holohover_props;
    ControlLQRSettings control_settings;

    Holohover holohover;

    Holohover::state_t<double> state;
    Eigen::Vector3d disturbance;
    Holohover::control_force_t<double> motor_velocities;
    // Holohover::control_acc_t<double> last_control_acc;
    Holohover::control_force_t<double> last_control_signal;
    holohover_msgs::msg::HolohoverState ref;
    Eigen::Matrix<double, Holohover::NA, Holohover::NX> K;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverStateDisturbanceStamped>::SharedPtr state_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverState>::SharedPtr reference_subscription;

    void init_topics();
    void init_timer();
    void publish_control();
    void state_callback(const holohover_msgs::msg::HolohoverStateDisturbanceStamped &state_msg);
    void ref_callback(const  holohover_msgs::msg::HolohoverState &pose);

    // //GS
    // Holohover::control_acc_t<double> u_acc_curr; //sent to hovercraft
    Holohover::control_acc_t<double> u_acc_bc_curr;
    Holohover::control_acc_t<double> u_acc_lqr_curr;

    //LQR logging
    std::ostringstream file_name_lqr;
    std::ofstream log_file_lqr;
};

#endif //HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_NODE_HPP
