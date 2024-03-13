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

#ifndef HOLOHOVER_GNC_DMPC_ADMM_NODE_HPP
#define HOLOHOVER_GNC_DMPC_ADMM_NODE_HPP

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/SparseLU"


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "holohover_common/models/holohover_model.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_admm_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "holohover_msgs/msg/holohover_trajectory.hpp"
#include "holohover_msgs/msg/holohover_laopt_speed_stamped.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
//#include "holohover_ocp.hpp"
#include "control_dmpc_settings.hpp"

#include <numeric> //accumulate
#include <vector>
#include <math.h> //pow, max
#include <iostream>
#include <fstream>
#include <thread>
#include <cfloat> //DBL_MAX

#include <iostream>
#include <iomanip>
#include <chrono>

#include "sProb.hpp"
#include "vcpy.hpp"

#include <qpOASES.hpp>
#include <casadi/casadi.hpp>

class HolohoverDmpcAdmmNode : public rclcpp::Node
{
public:
    static constexpr int N = 20;

public:
    HolohoverDmpcAdmmNode(const std::string& folder_name_sprob_, int my_id_, int Nagents_);
    ~HolohoverDmpcAdmmNode();
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

    int nx;
    int ng;
    int nh;
    int Ncons;
    sProb sprob;

    Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_bar;
    Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A;
    VectorXd g;
    VectorXd g_bar;
    VectorXd lbA;
    VectorXd ubA;
    VectorXd ub_vec;
    VectorXd lb_vec;

    qpOASES::Options myOptions;
    qpOASES::QProblem loc_prob;
    int nWSR;
    double rho;
    VectorXd lb;
    VectorXd ub; 

    //problem metadata
    Vector<bool, Eigen::Dynamic> isOriginal;    //nx x 1
    Vector<bool, Eigen::Dynamic> isCopy;        //nx x 1
    Vector<int, Eigen::Dynamic> numCopies;      //nx x 1 (how many copies there are of each original variable)    
    int N_og;                                   //Number of original variables this agent owns
    std::map<int,int> og_idx_to_idx;            //original variable index -> local variable index
    std::map<int,int> idx_to_og_idx;            //local variable index -> original variable index

    //ADMM iterates
    VectorXd z;                                 //nx x 1
    VectorXd zbar;                              //nx x 1
    VectorXd gam;                               //nx x 1

    vCpy* v_in;     //copy from in-neighbors       //N_in_neighbors x 1
    vCpy* v_out;    //copy for out-neighbors       //N_out_neighbors x 1
    std::vector<double>* XV;                       //N_og x n_out_neighbors(i)

    std::vector<int> in_neighbors;              //N_in_neighbors x 1
    std::vector<int> out_neighbors;             //N_out_neighbors x 1
    int N_in_neighbors;
    int N_out_neighbors;
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
    
    void init_coupling();   //extract coupling metadata from coupling matrices        
        
    void build_qp(const std::string& folder_name_sprob_); //construct the sProb
    
    int solve(unsigned int maxiter_, Eigen::VectorXd& zbar_);

    //before averaging
    void update_v_in();     //place z into v_in
    void send_vin_receive_vout(int64_t seq_number_);

    //after averaging
    void update_v_out();
    void send_vout_receive_vin(int64_t seq_number_);

    void init_comms();

    //communication
    VectorXi* v_in_msg_idx_first_received;
    VectorXi*  v_out_msg_idx_first_received;

    holohover_msgs::msg::HolohoverADMMStamped *v_in_msg;
    holohover_msgs::msg::HolohoverADMMStamped *v_in_msg_old;
    holohover_msgs::msg::HolohoverADMMStamped *v_out_msg;
    holohover_msgs::msg::HolohoverADMMStamped *v_out_msg_old;
    holohover_msgs::msg::HolohoverADMMStamped *v_in_msg_recv_buff;
    holohover_msgs::msg::HolohoverADMMStamped *v_out_msg_recv_buff;

    // rclcpp::Publisher<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr *v_in_publisher;
    // rclcpp::Publisher<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr *v_out_publisher;

    // rclcpp::Subscription<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr *v_in_subscriber;
    // rclcpp::Subscription<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr *v_out_subscriber;

    Eigen::Array<bool,Dynamic,1> received_vin;
    Eigen::Array<bool,Dynamic,1> received_vout;

    // void received_vin_callback(const holohover_msgs::msg::HolohoverADMMStamped &v_in_msg_, int in_neighbor_id_);
    // void received_vout_callback(const holohover_msgs::msg::HolohoverADMMStamped &v_out_msg_, int out_neighbor_id_);

    int update_g_beq(); 
    
    //GS END
};


#endif //HOLOHOVER_GNC_DMPC_ADMM_NODE_HPP