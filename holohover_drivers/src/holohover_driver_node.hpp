#ifndef HOLOHOVER_DRIVERS_HOLOHOVER_DRIVER_NODE_HPP
#define HOLOHOVER_DRIVERS_HOLOHOVER_DRIVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "msp.hpp"
#include "pmw3389dm.hpp"

#define UART_TIMEOUT 100

#define MOTOR_WATCHDOG_TIMEOUT 100
#define MOTOR_A_1  0
#define MOTOR_A_2  1
#define MOTOR_B_1  2
#define MOTOR_B_2  3
#define MOTOR_C_1  4
#define MOTOR_C_2  5

class HolohoverDriverNode : public rclcpp::Node
{
public:
    HolohoverDriverNode();
private:
    MSP msp;
    msp_motor_t motors;
    bool motor_are_reset = false;
    rclcpp::Time last_control_msg_time;

    PMW3389DM mouse_sensor;

    rclcpp::TimerBase::SharedPtr watchdog_timer;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_subscription;

    void watchdog_callback();
    void control_callback(const holohover_msgs::msg::HolohoverControlStamped& msg);

    void reset_motors();
};

#endif //HOLOHOVER_DRIVERS_HOLOHOVER_DRIVER_NODE_HPP
