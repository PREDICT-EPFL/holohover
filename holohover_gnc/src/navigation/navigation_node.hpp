#ifndef HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_NODE_HPP

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "holohover_msgs/msg/holohover_imu_stamped.hpp"
#include "holohover_msgs/msg/holohover_mouse_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "rigid_body_2d_ekf.hpp"
#include "navigation_settings.hpp"
#include "load_navigation_settings.hpp"

class HolohoverNavigationNode : public rclcpp::Node
{
public:
    HolohoverNavigationNode();
private:
    NavigationSettings navigation_settings;

    std::unique_ptr<holohover::RigidBody2DEKF> ekf;
    rclcpp::Time last_update;

    holohover_msgs::msg::HolohoverIMUStamped current_imu;
    bool received_imu = false;
    holohover_msgs::msg::HolohoverControlStamped current_control;
    bool received_control = false;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr state_acc_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverIMUStamped>::SharedPtr imu_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverMouseStamped>::SharedPtr mouse_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_subscription;

    void init_topics();
    void init_timer();
    void kalman_predict_step();
    void imu_callback(const holohover_msgs::msg::HolohoverIMUStamped &measurement);
    void mouse_callback(const holohover_msgs::msg::HolohoverMouseStamped &measurement);
    void pose_callback(const geometry_msgs::msg::PoseStamped &measurement);
    void control_callback(const holohover_msgs::msg::HolohoverControlStamped &control);
};


#endif //HOLOHOVER_GNC_HOLOHOVER_NAVIGATION_NODE_HPP
