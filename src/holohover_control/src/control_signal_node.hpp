#ifndef HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_NODE_HPP

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_common/riccati_solver.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "control_lqr_settings.hpp"

#define NB_SIGNALS 	16
#define NB_MOTORS 	6
#define CYCLE_TIME 	5000000 // in us, signal-on time + signal-off time
#define OFF_TIME 	1000000 // in us, signal-off time

class HolohoverControlSignalNode : public rclcpp::Node
{
public:
    HolohoverControlSignalNode();
private:
    HolohoverProps holohover_props;
    ControlLQRSettings control_settings;

    Holohover holohover;

    Holohover::state_t<double> state;
    geometry_msgs::msg::Pose2D ref;
    Eigen::Matrix<double, Holohover::NA, Holohover::NX> K;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_publisher;

    void init_topics();
    void init_timer();
    void publish_control();
    void comb2signal(Holohover::control_force_t<double>& u_signal, int comb);
    
};

#endif //HOLOHOVER_GNC_HOLOHOVER_CONTROL_LQR_NODE_HPP
