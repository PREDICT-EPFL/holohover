#ifndef HOLOHOVER_GNC_HOLOHOVER_CONTROL_TELEOP_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_CONTROL_TELEOP_NODE_HPP

#include <chrono>
#include <cmath>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "holohover_common/utils/holohover_props.hpp"

// Keyboard teleoperation in the body frame (no state estimate needed).
// Each key press commands an acceleration for `hold_time` seconds; holding the key
// keeps it active through the terminal's auto-repeat. Released keys stop the thrust,
// so the hovercraft keeps gliding: press the opposite key to brake.
class HolohoverControlTeleopNode : public rclcpp::Node
{
public:
    HolohoverControlTeleopNode();
    ~HolohoverControlTeleopNode() override;
private:
    HolohoverProps holohover_props;

    double period;
    double max_acc;
    double max_ang_acc;
    double hold_time;
    double scale;

    Holohover holohover;

    Holohover::control_acc_t<double> acc_cmd;
    rclcpp::Time last_key_time;

    struct termios original_termios;
    bool termios_saved;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_publisher;

    void init_terminal();
    void restore_terminal();
    void print_help();
    void print_status();
    void read_keys();
    void publish_control();
};

#endif //HOLOHOVER_GNC_HOLOHOVER_CONTROL_TELEOP_NODE_HPP
