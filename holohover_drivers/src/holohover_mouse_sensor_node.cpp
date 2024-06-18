#include "holohover_mouse_sensor_node.hpp"

HolohoverMouseSensorNode::HolohoverMouseSensorNode() :
    Node("mouse_sensor"),
    last_period_valid_measurement(false)
{
    std::string device = this->declare_parameter<std::string>("device");
    std::string n_reset_line_name = this->declare_parameter<std::string>("n_reset_line_name");
    double period = this->declare_parameter<double>("period", 0.01);

    if (!mouse_sensor.begin(device.c_str(), n_reset_line_name.c_str())) {
        throw std::runtime_error("Failed initializing mouse sensor");
    } else if (mouse_sensor.power_up_and_upload_firmware() < 0) {
        throw std::runtime_error("Failed uploading firmware to mouse sensor");
    }
    if (mouse_sensor.init_burst_read() < 0) {
        throw std::runtime_error("Failed initializing burst read");
    }

    mouse_timer = this->create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&HolohoverMouseSensorNode::mouse_callback, this));

    mouse_publisher = this->create_publisher<holohover_msgs::msg::HolohoverMouseStamped>("mouse", rclcpp::SensorDataQoS());
}

void HolohoverMouseSensorNode::mouse_callback()
{
    uint8_t burst_buffer[8];
    int retv = mouse_sensor.burst_read(burst_buffer);
    rclcpp::Time current_time = this->now();
    if (retv >= 0) {
        bool motion = (burst_buffer[0] & 0x80) > 0;
        bool lifted = (burst_buffer[0] & 0x08) > 0;

        if (lifted) {
            RCLCPP_INFO(this->get_logger(), "Holohover ist lifted");
            return;
        }

        holohover_msgs::msg::HolohoverMouseStamped mouse_msg;
        mouse_msg.header.frame_id = "body";
        mouse_msg.header.stamp = current_time;

        if (motion) {
            // movement count since last call
            int16_t delta_x = (int16_t) ((burst_buffer[3] << 8) | burst_buffer[2]);
            int16_t delta_y = (int16_t) ((burst_buffer[5] << 8) | burst_buffer[4]);

            // last term converts inch to meters
            double dist_x = (double) delta_x / 16000 * 0.0254;
            double dist_y = (double) delta_y / 16000 * 0.0254;
            double diff_time = (current_time - last_measurement_time).seconds();

            mouse_msg.v_x = dist_x / diff_time;
            mouse_msg.v_y = -dist_y / diff_time;
        } else {
            mouse_msg.v_x = 0;
            mouse_msg.v_y = 0;
        }

        if (last_period_valid_measurement) {
            mouse_publisher->publish(mouse_msg);
        }

        last_period_valid_measurement = true;
        last_measurement_time = current_time;
    } else {
        last_period_valid_measurement = false;
        RCLCPP_WARN(this->get_logger(), "Burst read failed");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverMouseSensorNode>());
    rclcpp::shutdown();
    return 0;
}
