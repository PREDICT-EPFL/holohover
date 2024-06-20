#include "holohover_fc_node.hpp"

HolohoverFCNode::HolohoverFCNode() :
    Node("fc")
{
    std::string device = this->declare_parameter<std::string>("fc_device");
    unsigned int baud = this->declare_parameter<int>("baud_rate", 1000000);

    bool enable_imu = this->declare_parameter<bool>("enable_imu", false);
    double imu_period = this->declare_parameter<double>("imu_period", 0.01);
    double diagnostics_period = this->declare_parameter<double>("diagnostics_period", 1);

    if (!msp.begin(device.c_str(), baud)) {
        throw std::runtime_error("Could not setup uart connection");
    }

    reset_motors();
    last_control_msg_time = this->now();

    watchdog_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&HolohoverFCNode::watchdog_callback, this));

    control_subscription = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(
            "control",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverFCNode::control_callback, this, std::placeholders::_1));

    if (enable_imu) {
        imu_timer = this->create_wall_timer(
                std::chrono::duration<double>(imu_period),
                std::bind(&HolohoverFCNode::imu_callback, this));

        imu_publisher = this->create_publisher<holohover_msgs::msg::HolohoverIMUStamped>("imu", rclcpp::SensorDataQoS());
    }

    diagnostics_timer = this->create_wall_timer(
            std::chrono::duration<double>(diagnostics_period),
            std::bind(&HolohoverFCNode::diagnostics_callback, this));
    voltage_publisher = this->create_publisher<std_msgs::msg::Float32>("voltage", 10);
}

void HolohoverFCNode::watchdog_callback()
{
    if (this->now() > last_control_msg_time + std::chrono::milliseconds(MOTOR_WATCHDOG_TIMEOUT)) {
        reset_motors();
    }
}

void HolohoverFCNode::control_callback(const holohover_msgs::msg::HolohoverControlStamped& msg)
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

void HolohoverFCNode::imu_callback()
{
    ssize_t msp_raw_imu_result = msp.request(MSP_RAW_IMU, reinterpret_cast<uint8_t*>(&imu), sizeof(imu), UART_TIMEOUT);
    if (msp_raw_imu_result >= 0) {
        holohover_msgs::msg::HolohoverIMUStamped imu_msg;
        imu_msg.header.frame_id = "body";
        imu_msg.header.stamp = this->now();

        imu_msg.acc.x = (double) imu.acc[1] / (512 * 4) * 9.81;
        imu_msg.acc.y = -((double) imu.acc[0] / (512 * 4) * 9.81);
        imu_msg.acc.z = -((double) imu.acc[2] / (512 * 4) * 9.81);

        imu_msg.gyro.x = (double) imu.gyro[1] / 32767 * 2000 / 180 * M_PI;
        imu_msg.gyro.y = -((double) imu.gyro[0] / 32767 * 2000 / 180 * M_PI);
        imu_msg.gyro.z = (double) imu.gyro[2] / 32767 * 2000 / 180 * M_PI;

        imu_publisher->publish(imu_msg);
    } else {
        RCLCPP_WARN(this->get_logger(), "MSP raw imu request failed.");
    }
}

void HolohoverFCNode::diagnostics_callback()
{
    ssize_t msp_battery_state_result = msp.request(MSP_BATTERY_STATE, reinterpret_cast<uint8_t*>(&battery_state), sizeof(battery_state), UART_TIMEOUT);
    if (msp_battery_state_result >= 0) {
        std_msgs::msg::Float32 msg;
        msg.data = (float) battery_state.voltage / 100;
        voltage_publisher->publish(msg);
    } else {
        RCLCPP_WARN(this->get_logger(), "MSP battery state request failed.");
    }
}

void HolohoverFCNode::reset_motors()
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
    rclcpp::spin(std::make_shared<HolohoverFCNode>());
    rclcpp::shutdown();
    return 0;
}
