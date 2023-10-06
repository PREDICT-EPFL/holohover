#ifndef HOLOHOVER_GNC_HOLOHOVER_MPC_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_MPC_NODE_HPP

// End user (level 1)

#include <Eigen/Dense>

#include "laopt/laopt.hpp"
#include "laopt/tools/control_problem_base.hpp"
#include "laopt/tools/multiple_shooting.hpp"
#include "laopt/solvers/sqp_solver.hpp"
#include "laopt/solvers/piqp_interface.hpp"

#include "holohover_common/models/holohover_model.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "holohover_msgs/msg/holohover_trajectory.hpp"
#include "holohover_msgs/msg/holohover_laopt_speed_stamped.hpp"
#include "holohover_ocp.hpp"
#include "control_mpc_settings.hpp"

class HolohoverControlMPCNode : public rclcpp::Node
{
public:
    static constexpr int N = 20;

    using Ocp = HolohoverOcp;
    using Transcription = laopt_tools::MultipleShooting<Ocp, N>;
    using Tape = laopt::TapeInfo<Transcription>;
    using OptProblem = laopt::Problem<Transcription>;
    using Solver = laopt::SQPSolver<OptProblem, laopt::PIQPSolver<OptProblem::scalar_t>>;
public:
    HolohoverControlMPCNode();
private:
    HolohoverProps holohover_props;
    ControlMPCSettings control_settings;

    Holohover holohover;
    Ocp ocp;
    Transcription transcription;
    Tape tape;
    OptProblem opt_problem;
    Solver solver;

    Holohover::state_t<double> state;
    holohover_msgs::msg::HolohoverState ref;
    //holohover_msgs::msg::HolohoverLaoptSpeedStamped speed;
    holohover_msgs::msg::HolohoverLaoptSpeedStamped speed_msg;
    //geometry_msgs::msg::Pose2D ref;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverLaoptSpeedStamped>::SharedPtr laopt_frequency_publisher;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_publisher;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverTrajectory>::SharedPtr HolohoverTrajectory_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverState>::SharedPtr reference_subscription;

    void init_topics();
    void init_timer();
    void publish_control();
    void publish_trajectory();
    void publish_laopt_speed(const long &duration_us );
    void state_callback(const holohover_msgs::msg::HolohoverStateStamped &state_msg);
    void ref_callback(const holohover_msgs::msg::HolohoverState &pose);
};


#endif //HOLOHOVER_GNC_HOLOHOVER_MPC_NODE_HPP
