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

#ifndef HOLOHOVER_GNC_DMPC_ADMM_LQR_NODE_HPP
#define HOLOHOVER_GNC_DMPC_ADMM_LQR_NODE_HPP

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/SparseLU"


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "std_msgs/msg/u_int64.hpp"

#include "holohover_common/models/holohover_model.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_admm_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_msgs/msg/holohover_dmpc_state_ref_stamped.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "holohover_msgs/msg/holohover_trajectory.hpp"
#include "holohover_msgs/msg/holohover_laopt_speed_stamped.hpp"
#include "control_dmpc_settings.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <numeric> //accumulate
#include <vector>
#include <math.h> //pow, max
#include <iostream>
#include <fstream>
#include <thread>
#include <cfloat> //DBL_MAX
#include <mutex>

#include <iostream>
#include <iomanip>
#include <chrono>

#include "sProb.hpp"
#include "vcpy.hpp"
#include "doptTimer.hpp"
#include "piqp/piqp.hpp"

#include "casadi/casadi.hpp"

#include <memory>

class HolohoverDmpcAdmmLqrNode : public rclcpp::Node
{
public:
    HolohoverDmpcAdmmLqrNode();
    ~HolohoverDmpcAdmmLqrNode();
private:
    HolohoverProps holohover_props;
    ControlDMPCSettings control_settings;

    Holohover holohover;
    
    Holohover::state_t<double> state;
    Eigen::VectorXd state_ref;   //GS: VectorXd, because different subsystems can have different state_ref dimension


    rclcpp::Publisher<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_publisher;
    rclcpp::Publisher<holohover_msgs::msg::HolohoverTrajectory>::SharedPtr HolohoverTrajectory_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverDmpcStateRefStamped>::SharedPtr reference_subscription;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr dmpc_trigger_subscription; //triggers publish_control()    
    

    Holohover::control_acc_t<double> u_acc_dmpc_curr;
    Holohover::control_acc_t<double> u_acc_dmpc_next;
    Holohover::control_acc_t<double> u_acc; //sent to hovercraft
    Holohover::control_acc_t<double> u_acc_lqr;
    Holohover::control_acc_t<double> u_acc_dmpc_curr_buff; //LQR stores u_acc_dmpc_curr here
    Holohover::control_force_t<double> motor_velocities;
    Holohover::control_force_t<double> last_control_signal;
    Holohover::control_force_t<double> u_signal;

    //OCP
    Eigen::VectorXd p; //parameters for OCP ([x0; u0; xd])
    int my_id;
    int Nagents;
    int nz;
    int ng;
    int nh;
    int Ncons;
    sProb sprob;
    Eigen::VectorXd g;
    Eigen::VectorXd lb;
    Eigen::VectorXd ub; 
    
    //problem metadata
    Eigen::Vector<bool, Eigen::Dynamic> isOriginal;    //nz x 1
    Eigen::Vector<bool, Eigen::Dynamic> isCopy;        //nz x 1
    Eigen::Vector<int, Eigen::Dynamic> numCopies;      //nz x 1 (how many copies there are of each original variable)    
    int N_og;                                   //Number of original variables this agent owns
    std::map<int,int> og_idx_to_idx;            //original variable index -> local variable index
    std::map<int,int> idx_to_og_idx;            //local variable index -> original variable index
    
    //ADMM
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_bar;
    Eigen::VectorXd g_bar;

    //ADMM iterates
    Eigen::VectorXd z;                                 //nz x 1
    Eigen::VectorXd zbar;                              //nz x 1
    Eigen::VectorXd gam;                               //nz x 1
    double rho;

    //ADMM coupling
    std::vector<vCpy> v_in;  //copy from in-neighbors       //N_in_neighbors x 1
    std::vector<vCpy> v_out; //copy for out-neighbors       //N_out_neighbors x 1
    std::vector<std::vector<double>> XV;                       //N_og x n_out_neighbors(i)
    std::vector<int> in_neighbors;              //N_in_neighbors x 1
    std::vector<int> out_neighbors;             //N_out_neighbors x 1
    int N_in_neighbors;
    int N_out_neighbors;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::SubscriptionOptions receive_vin_options;
    rclcpp::SubscriptionOptions receive_vout_options;
    rclcpp::CallbackGroup::SharedPtr receive_vin_cb_group;
    rclcpp::CallbackGroup::SharedPtr receive_vout_cb_group;  

    piqp::SparseSolver<double> loc_prob;

    bool dmpc_is_initialized;

    std::mutex state_mutex;
    std::mutex state_ref_mutex; 
    std::mutex u_acc_curr_mutex; //GS: remove? 
    std::mutex dmpc_lqr_mutex;
    
    void init_topics();
    void init_dmpc();
    void init_coupling();   //extract coupling metadata from coupling matrices 
    void init_comms();     
    void init_timer();
    void build_qp(); //construct the sProb
  

    //callbacks
    void state_callback(const holohover_msgs::msg::HolohoverStateStamped &state_msg);
    void ref_callback(const holohover_msgs::msg::HolohoverDmpcStateRefStamped &state_ref);
    void publish_control();
    void run_admm(const std_msgs::msg::UInt64 &publish_control_msg);
    rclcpp::SubscriptionOptions run_admm_options;
    rclcpp::SubscriptionOptions publish_control_options;
    rclcpp::CallbackGroup::SharedPtr run_admm_cb_group;
    rclcpp::CallbackGroup::SharedPtr publish_control_cb_group;
    
    void publish_trajectory();

    //LQR
    int lqr_step;
    Holohover::state_t<double> predicted_state;
    Holohover::state_t<double> state_at_ocp_solve;
    Holohover::state_t<double> state_at_ocp_solve_buff;
    bool new_dmpc_acc_available;
    
    //DMPC loop
    void update_setpoint_in_ocp();
    int solve(unsigned int maxiter_);
    void get_u_acc_from_sol();
    void convert_u_acc_to_u_signal();

    //before averaging
    void update_v_in();     //place z into v_in
    void send_vin_receive_vout();

    //after averaging
    void update_v_out();
    void send_vout_receive_vin();

    //ADMM communication
    std::vector<std::vector<int>> v_in_msg_idx_first_received;
    std::vector<std::vector<int>> v_out_msg_idx_first_received;

    std::vector<holohover_msgs::msg::HolohoverADMMStamped> v_in_msg;
    std::vector<holohover_msgs::msg::HolohoverADMMStamped> v_out_msg;
    std::vector<holohover_msgs::msg::HolohoverADMMStamped> v_in_msg_recv_buff;
    std::vector<holohover_msgs::msg::HolohoverADMMStamped> v_out_msg_recv_buff;

    std::vector<rclcpp::Publisher<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr> v_in_publisher;
    std::vector<rclcpp::Publisher<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr> v_out_publisher;

    std::vector<rclcpp::Subscription<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr> v_in_subscriber;
    std::vector<rclcpp::Subscription<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr> v_out_subscriber;

    Eigen::Array<bool,Dynamic,1> received_vin;
    Eigen::Array<bool,Dynamic,1> received_vout;

    void received_vin_callback(const holohover_msgs::msg::HolohoverADMMStamped &v_in_msg_, int in_neighbor_id_); //
    void received_vout_callback(const holohover_msgs::msg::HolohoverADMMStamped &v_out_msg_, int out_neighbor_id_); //
    std::vector<std::function<void(const holohover_msgs::msg::HolohoverADMMStamped &v_in_msg_)>> bound_received_vin_callback;
    std::vector<std::function<void(const holohover_msgs::msg::HolohoverADMMStamped &v_out_msg_)>> bound_received_vout_callback;

    //ADMM logging
    doptTimer admm_timer;
    doptTimer loc_timer;        //time for solving the local QP
    doptTimer iter_timer;       //time for running one iteration
    doptTimer z_comm_timer;     //time for communicating z in one iteration
    doptTimer zbar_comm_timer;  //time for communicating zbar in one iteration
    doptTimer send_vin_timer;
    doptTimer receive_vout_timer;
    Eigen::MatrixXd x_log;
    Eigen::MatrixXd u_log;
    Eigen::MatrixXd xd_log;
    int mpc_step;
    int log_buffer_size; //number of MPC steps to store before writing to log
    int logged_mpc_steps; //number of MPC steps that have already been logged
    void print_time_measurements();
    void clear_time_measurements();
    void reserve_time_measurements(unsigned int new_cap);
    std::ostringstream file_name;
    std::ofstream log_file;   

    //helper
    Eigen::MatrixXd casadi2Eigen ( const casadi::DM& A );
    Eigen::VectorXd casadi2EigenVector ( const casadi::DM& A );
    casadi::DM Eigen2casadi( const Eigen::VectorXd& in);
    
};


#endif //HOLOHOVER_GNC_DMPC_ADMM_LQR_NODE_HPP