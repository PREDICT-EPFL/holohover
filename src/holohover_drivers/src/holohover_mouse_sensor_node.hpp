#ifndef HOLOHOVER_DRIVERS_HOLOHOVER_MOUSE_SENSOR_NODE_HPP
#define HOLOHOVER_DRIVERS_HOLOHOVER_MOUSE_SENSOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "holohover_msgs/msg/holohover_mouse_stamped.hpp"
#include "pmw3389dm.hpp"

class HolohoverMouseSensorNode : public rclcpp::Node
{
public:
    HolohoverMouseSensorNode();
private:
    PMW3389DM mouse_sensor;
    rclcpp::Time last_measurement_time;
    bool last_period_valid_measurement;

    rclcpp::TimerBase::SharedPtr mouse_timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverMouseStamped>::SharedPtr mouse_publisher;

    void mouse_callback();
};

#endif //HOLOHOVER_DRIVERS_HOLOHOVER_MOUSE_SENSOR_NODE_HPP
