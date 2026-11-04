#include "control_teleop_node.hpp"

#include <cstdio>
#include <unistd.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

HolohoverControlTeleopNode::HolohoverControlTeleopNode() :
        Node("control_teleop"),
        holohover_props(load_holohover_pros(declare_parameter<std::string>(
                "holohover_props_file",
                ament_index_cpp::get_package_share_directory("holohover_utils") + "/config/common/holohover_params.yaml"))),
        period(declare_parameter<double>("period", 0.02)),             // [s], firmware watchdog is 100 ms
        max_acc(declare_parameter<double>("max_acc", 0.5)),            // [m/s^2] at 100% scale
        max_ang_acc(declare_parameter<double>("max_ang_acc", 5.0)),    // [rad/s^2] at 100% scale
        hold_time(declare_parameter<double>("hold_time", 0.6)),        // [s], longer than the key auto-repeat delay
        scale(declare_parameter<double>("initial_scale", 0.3)),
        holohover(holohover_props, period),
        termios_saved(false)
{
    acc_cmd.setZero();
    last_key_time = this->now();

    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControlStamped>(
            "control",
            rclcpp::SensorDataQoS());

    init_terminal();
    print_help();

    timer = this->create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&HolohoverControlTeleopNode::publish_control, this));
}

HolohoverControlTeleopNode::~HolohoverControlTeleopNode()
{
    restore_terminal();
}

void HolohoverControlTeleopNode::init_terminal()
{
    if (!isatty(STDIN_FILENO)) {
        RCLCPP_ERROR(get_logger(), "stdin is not a terminal: run this node with `ros2 run`, not from a launch file.");
        return;
    }
    tcgetattr(STDIN_FILENO, &original_termios);
    termios_saved = true;

    // raw, non-blocking input: no line buffering, no echo
    struct termios raw = original_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

void HolohoverControlTeleopNode::restore_terminal()
{
    if (termios_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
        termios_saved = false;
    }
}

void HolohoverControlTeleopNode::print_help()
{
    printf("\nHolohover keyboard teleop (body frame), publishing on %s\n"
           "---------------------------\n"
           "        w                    q / e : rotate left / right\n"
           "   a    s    d               + / - : thrust scale up / down\n"
           "                             space : stop motors\n"
           "w/s: forward/back  a/d: left/right   Ctrl-C : quit\n\n",
           control_publisher->get_topic_name());
    print_status();
}

void HolohoverControlTeleopNode::print_status()
{
    printf("\rscale %3.0f%%  (max acc %.2f m/s^2, max ang acc %.2f rad/s^2)   ",
           scale * 100, scale * max_acc, scale * max_ang_acc);
    fflush(stdout);
}

void HolohoverControlTeleopNode::read_keys()
{
    char c;
    while (read(STDIN_FILENO, &c, 1) == 1) {
        Holohover::control_acc_t<double> dir;
        dir.setZero();
        switch (c) {
            case 'w': dir << 1, 0, 0; break;
            case 's': dir << -1, 0, 0; break;
            case 'a': dir << 0, 1, 0; break;
            case 'd': dir << 0, -1, 0; break;
            case 'q': dir << 0, 0, 1; break;
            case 'e': dir << 0, 0, -1; break;
            case '+': case '=':
                scale = std::min(1.0, scale + 0.1);
                print_status();
                continue;
            case '-': case '_':
                scale = std::max(0.1, scale - 0.1);
                print_status();
                continue;
            case ' ':
                acc_cmd.setZero();
                continue;
            default:
                continue;
        }
        acc_cmd << scale * max_acc * dir(0), scale * max_acc * dir(1), scale * max_ang_acc * dir(2);
        last_key_time = this->now();
    }
}

void HolohoverControlTeleopNode::publish_control()
{
    read_keys();

    if ((this->now() - last_key_time).seconds() > hold_time) {
        acc_cmd.setZero();
    }

    Holohover::control_force_t<double> u_signal;
    u_signal.setZero();
    if (!acc_cmd.isZero()) {
        // zero state => yaw = 0, so the requested acceleration is expressed in the body frame
        Holohover::state_t<double> state;
        state.setZero();
        Holohover::control_force_t<double> u_force;
        holohover.control_acceleration_to_force(state, acc_cmd, u_force);
        holohover.thrust_to_signal(u_force, u_signal);
        u_signal = u_signal.cwiseMax(holohover_props.idle_signal).cwiseMin(1);
    }

    holohover_msgs::msg::HolohoverControlStamped control_msg;
    control_msg.header.frame_id = "body";
    control_msg.header.stamp = this->now();
    control_msg.motor_a_1 = u_signal(0);
    control_msg.motor_a_2 = u_signal(1);
    control_msg.motor_b_1 = u_signal(2);
    control_msg.motor_b_2 = u_signal(3);
    control_msg.motor_c_1 = u_signal(4);
    control_msg.motor_c_2 = u_signal(5);
    control_publisher->publish(control_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HolohoverControlTeleopNode>();
    rclcpp::spin(node);
    // no final zero command needed: the firmware watchdog stops the motors 100 ms after the last message
    rclcpp::shutdown();
    return 0;
}
