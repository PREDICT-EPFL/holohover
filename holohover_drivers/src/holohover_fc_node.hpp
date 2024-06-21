#ifndef HOLOHOVER_DRIVERS_HOLOHOVER_FC_NODE_HPP
#define HOLOHOVER_DRIVERS_HOLOHOVER_FC_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_msgs/msg/holohover_imu_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "msp.hpp"

#define UART_TIMEOUT 100

#define MOTOR_WATCHDOG_TIMEOUT 500
#define MOTOR_A_1  0
#define MOTOR_A_2  1
#define MOTOR_B_1  2
#define MOTOR_B_2  3
#define MOTOR_C_1  4
#define MOTOR_C_2  5

class HolohoverFCNode : public rclcpp::Node
{
public:
    HolohoverFCNode();
    void reset_motors();

private:
    MSP msp;
    msp_motor_t motors;
    msp_raw_imu_t imu;
    msp_battery_state_t battery_state;
    bool motor_are_reset = false;
    rclcpp::Time last_control_msg_time;

    rclcpp::TimerBase::SharedPtr watchdog_timer;
    rclcpp::TimerBase::SharedPtr imu_timer;
    rclcpp::TimerBase::SharedPtr diagnostics_timer;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_subscription;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverIMUStamped>::SharedPtr imu_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;

    void watchdog_callback();
    void control_callback(const holohover_msgs::msg::HolohoverControlStamped& msg);

    void imu_callback();
    void diagnostics_callback();
};

#endif //HOLOHOVER_DRIVERS_HOLOHOVER_FC_NODE_HPP
