#ifndef HOLOHOVER_GNC_HOLOHOVER_SIMULATION_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_SIMULATION_NODE_HPP

#include <chrono>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_imu_stamped.hpp"
#include "holohover_msgs/msg/holohover_mouse_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_gnc/models/holohover_model.hpp"
#include "simulation_settings.hpp"
#include "holohover_gnc/utils/load_holohover_props.hpp"
#include "load_simulation_settings.hpp"

class HolohoverSimulationNode : public rclcpp::Node
{
public:
    HolohoverSimulationNode();
private:
    HolohoverProps holohover_props;
    SimulationSettings simulation_settings;

    Holohover holohover;

    std::mt19937 random_engine;

    Holohover::state_t<double> state;
    holohover_msgs::msg::HolohoverControlStamped current_control;
    Holohover::control_acc_t<double> current_control_acc;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr imu_timer;
    rclcpp::TimerBase::SharedPtr mouse_timer;
    rclcpp::TimerBase::SharedPtr pose_timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_publisher;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverIMUStamped>::SharedPtr imu_publisher;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverMouseStamped>::SharedPtr mouse_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_subscription;

    void init_topics();
    void init_timers();
    void calculate_control_acc();
    void simulate_forward_callback();
    void imu_callback();
    void mouse_callback();
    void pose_callback();
    void control_callback(const holohover_msgs::msg::HolohoverControlStamped &control);
};

#endif //HOLOHOVER_GNC_HOLOHOVER_SIMULATION_NODE_HPP
