#ifndef HOLOHOVER_GNC_HOLOHOVER_MPC_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_MPC_NODE_HPP

// End user (level 1)

#include <Eigen/Dense>

#include "holohover_common/models/holohover_model.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "holohover_msgs/msg/holohover_trajectory.hpp"
#include "holohover_msgs/msg/holohover_laopt_speed_stamped.hpp"
#include "control_mpc_settings.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "casadi/casadi.hpp"

using namespace casadi;

class HolohoverControlMPCNode : public rclcpp::Node
{
public:
    static constexpr int N = 20;
    static constexpr int nx = 6;
    static constexpr int nu = 6;

    HolohoverControlMPCNode();
     

private:
    HolohoverProps holohover_props;
    ControlMPCSettings control_settings;
    Opti opti;
    Holohover holohover;
    MX x;
    MX u;
    MX x_ref;
    MX x0;
    MX s;
    DM x_opt;
    DM u_opt;

    Holohover::state_t<double> state;
    holohover_msgs::msg::HolohoverState ref;
    //holohover_msgs::msg::HolohoverLaoptSpeedStamped speed;
    holohover_msgs::msg::HolohoverLaoptSpeedStamped speed_msg;
    //geometry_msgs::msg::Pose2D ref;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_publisher;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverTrajectory>::SharedPtr HolohoverTrajectory_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverState>::SharedPtr reference_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr puck_subscription;

    void init_topics();
    void init_timer();
    void publish_control();
    void publish_trajectory();
    void state_callback(const holohover_msgs::msg::HolohoverStateStamped &state_msg);
    void ref_callback(const holohover_msgs::msg::HolohoverState &pose);
    void puck_pose_callback(const geometry_msgs::msg::PoseStamped &puck_pose);


};


#endif //HOLOHOVER_GNC_HOLOHOVER_MPC_NODE_HPP
