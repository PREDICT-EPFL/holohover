#include "control_driver_node.hpp"

HolohoverControlDriverNode::HolohoverControlDriverNode() :
    Node("control_driver")
{
    std::string device = this->declare_parameter<std::string>("device");
    unsigned int baud = this->declare_parameter<int>("baud_rate", 1000000);

    if (!msp.begin(device.c_str(), baud)) {
        throw std::runtime_error("Could not setup uart connection");
    }

    reset_motors();
    last_control_msg_time = this->now();

    watchdog_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&HolohoverControlDriverNode::watchdog_callback, this));

    control_subscription = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(
            "control",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverControlDriverNode::control_callback, this, std::placeholders::_1));
}

void HolohoverControlDriverNode::watchdog_callback()
{
    if (this->now() > last_control_msg_time + std::chrono::milliseconds(MOTOR_WATCHDOG_TIMEOUT)) {
        reset_motors();
    }
}

void HolohoverControlDriverNode::control_callback(const holohover_msgs::msg::HolohoverControlStamped& msg)
{
    motors.motor[MOTOR_A_1] = 1000 + (int)(1000 * msg.motor_a_1);
    motors.motor[MOTOR_A_2] = 1000 + (int)(1000 * msg.motor_a_2);
    motors.motor[MOTOR_B_1] = 1000 + (int)(1000 * msg.motor_b_1);
    motors.motor[MOTOR_B_2] = 1000 + (int)(1000 * msg.motor_b_2);
    motors.motor[MOTOR_C_1] = 1000 + (int)(1000 * msg.motor_c_1);
    motors.motor[MOTOR_C_2] = 1000 + (int)(1000 * msg.motor_c_2);

    if (msp.command(MSP_SET_MOTOR, reinterpret_cast<uint8_t*>(&motors), sizeof(motors), UART_TIMEOUT) < 0) {
        RCLCPP_ERROR(this->get_logger(), "MSP set motor command failed.");
    }

    last_control_msg_time = this->now();
    motor_are_reset = false;
}

void HolohoverControlDriverNode::reset_motors()
{
    if (!motor_are_reset) {
        for (int i = 0; i < MSP_MAX_SUPPORTED_MOTORS; i++) {
            motors.motor[i] = 1000;
        }
        RCLCPP_INFO(this->get_logger(), "Init motors to 0.");
        if (msp.command(MSP_SET_MOTOR, reinterpret_cast<uint8_t*>(&motors), sizeof(motors), UART_TIMEOUT) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Initial MSP set motor command failed.");
        } else {
            motor_are_reset = true;
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlDriverNode>());
    rclcpp::shutdown();
    return 0;
}
