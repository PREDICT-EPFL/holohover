#ifndef HOLOHOVER_GNC_HOLOHOVER_MPC_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_MPC_NODE_HPP

// End user (level 1)

#include <Eigen/Dense>
#include <tuple>
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
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace casadi;

class HolohoverControlMPCNode : public rclcpp::Node
{
public:
    static constexpr int N = 20;
    static constexpr int nx = 4;
    static constexpr int nu = 2;
    static constexpr int na = 3;

    HolohoverControlMPCNode();
     

private:
    HolohoverProps holohover_props;
    ControlMPCSettings control_settings;
    Opti opti;
    Holohover holohover;
    MX x;
    MX u;
    MX x_ref;
    MX z_all;
    MX x0;
    MX s;
    MX puck_state;
    DM x_opt;
    DM u_opt;
    std::vector<casadi::MX> strike_trajectory;
    std::vector<casadi::MX> strike_vec_trajectory;

    double delta_t = 1.0 / N; // seconds
    std::tuple<double, double> home_pos;
    std::tuple<double, double> goal_pos;
    MX unit_dir_k;
    MX is_away;
    MX momentum_rewards;
    MX distance_cost;

    // Smith predictor
    std::deque<DM> control_history;
    double delay_seconds = control_settings.controller_delay;
    int delay_steps = static_cast<int>(delay_seconds / control_settings.period);

    // Puck information
    geometry_msgs::msg::Point last_position;
    rclcpp::Time last_time;
    bool first_callback = true;
    bool state_ready = false;
    bool puck_ready = false;
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
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub_;

    void init_topics();
    void init_timer();
    void publish_control();
    void publish_trajectory();
    void state_callback(const holohover_msgs::msg::HolohoverStateStamped &state_msg);
    void ref_callback(const holohover_msgs::msg::HolohoverState &pose);
    void puck_pose_callback(const geometry_msgs::msg::PoseStamped &puck_pose);
    void publish_strike_trajectory(const std::vector<casadi::DM>& trajectory);
    void publish_dual_trajectories(const casadi::DM& x_lti, const casadi::DM& x_rk4);

    void setup_ipopt(ControlMPCSettings control_settings);
    void setup_ipopt_old(ControlMPCSettings control_settings);
    void setup_hpipm(ControlMPCSettings control_settings);
    void setup_tracking(ControlMPCSettings control_settings);
};


#endif //HOLOHOVER_GNC_HOLOHOVER_MPC_NODE_HPP
