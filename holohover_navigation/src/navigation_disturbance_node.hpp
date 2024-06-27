#ifndef HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_DISTURBANCE_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_DISTURBANCE_NODE_HPP

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "holohover_msgs/msg/holohover_imu_stamped.hpp"
#include "holohover_msgs/msg/holohover_mouse_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_state_disturbance_stamped.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "disturbance_holohover_ekf.hpp"
#include "navigation_settings.hpp"

class HolohoverNavigationDisturbanceNode : public rclcpp::Node
{
public:
    HolohoverNavigationDisturbanceNode();
private:
    HolohoverProps holohover_props;
    NavigationSettings navigation_settings;

    std::unique_ptr<holohover::DisturbanceHolohoverEKF> ekf;
    rclcpp::Time last_update;

    holohover_msgs::msg::HolohoverIMUStamped current_imu;
    bool received_imu = false;
    holohover_msgs::msg::HolohoverControlStamped current_control;
    bool received_control = false;
    rclcpp::Time last_control_msg_time;

    rclcpp::TimerBase::SharedPtr watchdog_timer;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_publisher;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverStateDisturbanceStamped>::SharedPtr state_disturbance_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverIMUStamped>::SharedPtr imu_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverMouseStamped>::SharedPtr mouse_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_subscription;

    void init_topics();
    void init_timer();
    void watchdog_callback();
    void kalman_predict_step();
    void imu_callback(const holohover_msgs::msg::HolohoverIMUStamped &measurement);
    void mouse_callback(const holohover_msgs::msg::HolohoverMouseStamped &measurement);
    void pose_callback(const geometry_msgs::msg::PoseStamped &measurement);
    void control_callback(const holohover_msgs::msg::HolohoverControlStamped &control);
};


#endif //HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_DISTURBANCE_NODE_HPP
