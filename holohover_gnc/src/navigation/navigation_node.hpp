#ifndef HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_NODE_HPP

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "holohover_msgs/msg/holohover_measurement.hpp"
#include "holohover_msgs/msg/holohover_control.hpp"
#include "holohover_msgs/msg/holohover_state.hpp"
#include "holohover_gnc/models/holohover_model.hpp"
#include "holohover_ekf.hpp"
#include "navigation_settings.hpp"
#include "../utils/load_holohover_props.hpp"
#include "load_navigation_settings.hpp"

class HolohoverNavigationNode : public rclcpp::Node
{
public:
    HolohoverNavigationNode();
private:
    HolohoverProps holohover_props;
    NavigationSettings navigation_settings;

    Holohover holohover;
    HolohoverEKF kalman;

    holohover_msgs::msg::HolohoverMeasurement current_imu;
    bool received_imu = false;
    holohover_msgs::msg::HolohoverControl current_control;
    bool received_control = false;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverState>::SharedPtr state_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr comp_time_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverMeasurement>::SharedPtr imu_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControl>::SharedPtr control_subscription;

    void init_topics();
    void init_timer();
    void kalman_predict_step();
    void imu_callback(const holohover_msgs::msg::HolohoverMeasurement &measurement);
    void control_callback(const holohover_msgs::msg::HolohoverControl &control);
};


#endif //HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_NODE_HPP
