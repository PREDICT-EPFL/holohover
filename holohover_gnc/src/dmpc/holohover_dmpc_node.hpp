/*MIT License
Copyright (c) 2023 Goesta Stomberg, Henrik Ebel, Timm Faulwasser, Peter Eberhard
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef HOLOHOVER_GNC_HOLOHOVER_DMPC_NODE_HPP
#define HOLOHOVER_GNC_HOLOHOVER_DMPC_NODE_HPP

// End user (level 1)
#include "Eigen/Dense"

#include "holohover_common/models/holohover_model.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_admm_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "holohover_msgs/msg/holohover_trajectory.hpp"
#include "holohover_msgs/msg/holohover_laopt_speed_stamped.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
//#include "holohover_ocp.hpp"
#include "control_dmpc_settings.hpp"

#include "holohover_admm_node.hpp"

class HolohoverControlDMPCNode : public rclcpp::Node
{
public:
    static constexpr int N = 20;

public:
    HolohoverControlDMPCNode(const std::string& folder_name_sprob_, int my_id_, int Nagents_);
    ~HolohoverControlDMPCNode();
private:
    HolohoverProps holohover_props;
    ControlDMPCSettings control_settings;

    Holohover holohover;
    
    Holohover::state_t<double> state;       //GS: m_x_meas
    
    holohover_msgs::msg::HolohoverState ref;
    //holohover_msgs::msg::HolohoverLaoptSpeedStamped speed;
    holohover_msgs::msg::HolohoverLaoptSpeedStamped speed_msg;
    //geometry_msgs::msg::Pose2D ref;

    rclcpp::TimerBase::SharedPtr timer;
    //rclcpp::Publisher<holohover_msgs::msg::HolohoverLaoptSpeedStamped>::SharedPtr laopt_frequency_publisher;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_publisher;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverTrajectory>::SharedPtr HolohoverTrajectory_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverState>::SharedPtr reference_subscription;

    
    //GS BEGIN
    Eigen::VectorXd state_ref;   //GS: m_xd
    Eigen::VectorXd p; //parameters for OCP ([x0; u0; xd])
    Holohover::control_acc_t<double> u_acc_curr;
    Holohover::control_acc_t<double> u_acc_next; //GS: m_u1
    Holohover::control_force_t<double> motor_velocities;
    Holohover::control_force_t<double> last_control_signal;
    Holohover::control_force_t<double> u_signal;

    VectorXd sol; //GS: OCP solution
    int my_id; //GS: move to config?
    int Nagents;
    std::string folder_name_sprob; //GS: move to config?
    HolohoverControlADMMNode* admm;
    //GS END
    
    
    
    void init_topics();
    void init_timer();
    void publish_control();
    void publish_trajectory();
    void publish_laopt_speed(const long &duration_us );
    void state_callback(const holohover_msgs::msg::HolohoverStateStamped &state_msg);
    void ref_callback(const holohover_msgs::msg::HolohoverState &pose);

    //GS BEGIN
    void set_state_in_ocp();
    void set_u_acc_curr_in_ocp();
    void get_u_acc_from_sol();
    void update_setpoint_in_ocp();
    void init_dmpc();
    void convert_u_acc_to_u_signal();
    Eigen::MatrixXd casadi2Eigen ( const casadi::DM& A );
    Eigen::VectorXd casadi2EigenVector ( const casadi::DM& A );
    casadi::DM Eigen2casadi( const Eigen::VectorXd& in);
    //GS END
};


#endif //HOLOHOVER_GNC_HOLOHOVER_DMPC_NODE_HPP