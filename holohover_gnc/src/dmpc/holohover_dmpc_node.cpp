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

#include <iostream>
#include <iomanip>
#include <chrono>

#include "holohover_dmpc_node.hpp"


using namespace std::chrono;

HolohoverControlDMPCNode::HolohoverControlDMPCNode(const std::string& folder_name_sprob_, int my_id_, int Nagents_) :
        Node("control_dmpc"),
        holohover_props(load_holohover_pros(*this)),
        control_settings(load_control_dmpc_settings(*this)),
        holohover(holohover_props, control_settings.period)
{
    folder_name_sprob = folder_name_sprob_;
    my_id = my_id_;
    Nagents = Nagents_;    
    admm = nullptr;

    // init state
    state.setZero();
    u_acc_curr.setZero();
    u_acc_next.setZero();
    motor_velocities.setZero();
    last_control_signal.setZero();
    u_signal.setZero();


    p = Eigen::VectorXd::Zero(control_settings.nx+control_settings.nu+control_settings.nxd);
    state_ref = p.segment(control_settings.nx+control_settings.nu,control_settings.nxd); //todo

    build_qp(folder_name_sprob_);

    nx = sprob_.H[my_id].rows();
    ng = sprob_.Aeq[my_id].rows();
    nh = sprob_.Aineq[my_id].rows();

    std::cout << "nx = " << nx << " , " << "ng = " << ng << " , " << "nh = " << nh << std::endl;

    Ncons = sprob_.A[my_id].rows();
    rho = rho_;
    H_bar = sprob_.H[my_id] + rho * MatrixXd::Identity(nx,nx);
    g = sprob_.g[my_id];
    A = MatrixXd::Zero(ng+nh,nx);
    A.block(0,0,ng,nx) = sprob_.Aeq[my_id];
    
    if (nh > 0){
        A.block(ng,0,nh,nx) = sprob_.Aineq[my_id];
    }

    lbA = VectorXd::Zero(ng+nh);
    ubA = VectorXd::Zero(ng+nh);
    lbA.block(0,0,ng,1) = sprob_.beq[my_id];
    ubA.block(0,0,ng,1) = sprob_.beq[my_id];

    if (nh > 0){
        lbA.block(ng,0,nh,1).setConstant(-pow(10,20));  
        ubA.block(ng,0,nh,1) = sprob_.bineq[my_id];
    }

    z       = VectorXd::Zero(nx);
    zbar   = VectorXd::Zero(nx);
    gam     = VectorXd::Zero(nx);
    ub = sprob_.ub[my_id];
    lb = sprob_.lb[my_id];

    myOptions.printLevel = qpOASES::PL_NONE; // other values: PL_NONE, PL_LOW, PL_MEDIUM, PL_HIGH, PL_TABULAR, PL_DEBUG_ITER
 
    nWSR = 1000;
    g_bar = g + gam - rho*zbar;

    std::cout << "initializing qpOASES" << std::endl;

    loc_prob = qpOASES::QProblem(nx,ng+nh);
    loc_prob.setOptions(myOptions);
    loc_prob.init(H_bar.data(), g_bar.data(), A.data(),
                           lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR);    

    isOriginal.resize(nx);
    isOriginal.setConstant(false);
    isCopy.resize(nx);
    isCopy.setConstant(false);

    out_neighbors = std::vector<int>(0);
    in_neighbors = std::vector<int>(0);
    numCopies = VectorXi::Zero(nx);    
    
    init_coupling();

    XV = new std::vector<double>[N_og];
    for (int i = 0; i < N_og; i++){
        auto idx = og_idx_to_idx.find(i);
        XV[i].resize(numCopies(idx->second)+1); //XV stores local original variable and copies from out-neighbors
    }

    //initialize comms
    // v_out
    v_out_msg = new holohover_msgs::msg::HolohoverADMMStamped[N_out_neighbors];
    // v_out_subscriber = new rclcpp::Subscription<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr[N_out_neighbors];
    // v_out_publisher = new rclcpp::Publisher<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr[N_out_neighbors];

    // for (int i = 0; i < N_out_neighbors; i++){
    //     int neighbor_id = v_out[i].copying_agent;       

    //     v_out_msg[i].val_length = v_out[i].nv;
    //     v_out_msg[i].idx_length = v_out[i].nv;
    //     v_out_msg[i].value.resize(v_out_msg[i].val_length);
    //     v_out_msg[i].idx.resize(v_out_msg[i].idx_length);
    //     v_out_msg[i].seq_number = 0;
    //     for (int idx_row = 0; idx_row<v_out_msg[i].val_length; idx_row++) {
    //         v_out_msg[i].value[idx_row] = std::numeric_limits<double>::quiet_NaN();
    //     }
    //     v_out_msg[i].id_sender = my_id;

    //     std::string v_out_pub_topic = "/agent";
    //     v_out_pub_topic.append(std::to_string(my_id));
    //     v_out_pub_topic.append("/og/foragent");
    //     v_out_pub_topic.append(std::to_string(neighbor_id));
    //     v_out_publisher[i] = this->create_publisher<holohover_msgs::msg::HolohoverADMMStamped>(
    //         v_out_pub_topic,
    //         rclcpp::SensorDataQoS());  
    
    //     std::string v_out_sub_topic = "/agent";
    //     v_out_sub_topic.append(std::to_string(neighbor_id));
    //     v_out_sub_topic.append("copyofagent");
    //     v_out_sub_topic.append(std::to_string(my_id));
    //     v_out_subscriber[i] = this->create_subscription<holohover_msgs::msg::HolohoverADMMStamped>(
    //                         v_out_sub_topic, 10,
    //                         std::bind(&HolohoverControlADMMNode::received_vout_callback, this, std::placeholders::_1)); //i
    // }

    // v_in
    // v_in_msg = new holohover_msgs::msg::HolohoverADMMStamped[N_in_neighbors];
    // v_in_subscriber = new rclcpp::Subscription<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr[N_in_neighbors];
    // v_in_publisher = new rclcpp::Publisher<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr[N_in_neighbors];

    // for (int i = 0; i < N_in_neighbors; i++){
    //     int neighbor_id = v_in[i].original_agent;

    //     v_in_msg[i].val_length = v_in[i].nv;
    //     v_in_msg[i].value.resize(v_in_msg[i].val_length);
    //     v_in_msg[i].idx_length = v_in[i].nv;
    //     v_in_msg[i].idx.resize(v_in_msg[i].idx_length);
    //     Eigen::VectorXi::Map(&v_in_msg[i].idx[0], v_in[i].og_idx.size()) = v_in[i].og_idx;
    //     v_in_msg[i].seq_number = 0;
    //     for (int idx_row = 0; idx_row<v_in_msg[i].val_length; idx_row++) {
    //         v_in_msg[i].value[idx_row] = std::numeric_limits<double>::quiet_NaN();
    //     }
    //     v_in_msg[i].id_sender = my_id;

    //     std::string v_in_pub_topic = "/agent";
    //     v_in_pub_topic.append(std::to_string(my_id));
    //     v_in_pub_topic.append("copyofagent");
    //     v_in_pub_topic.append(std::to_string(neighbor_id));
    //     v_in_publisher[i] = this->create_publisher<holohover_msgs::msg::HolohoverADMMStamped>(
    //         v_in_pub_topic,
    //         rclcpp::SensorDataQoS());

    //     std::string v_in_sub_topic = "/agent";
    //     v_in_sub_topic.append(std::to_string(neighbor_id));
    //     v_in_sub_topic.append("/og/foragent");
    //     v_in_sub_topic.append(std::to_string(my_id));
    //     v_in_subscriber[i] = this->create_subscription<holohover_msgs::msg::HolohoverADMMStamped>(
    //                         v_in_sub_topic, 10,
    //                         std::bind(&HolohoverControlADMMNode::received_vin_callback, this, std::placeholders::_1));  //i  
    // }

    v_in_msg_idx_first_received = new VectorXi[N_in_neighbors];
    v_out_msg_idx_first_received = new VectorXi[N_out_neighbors];





    init_topics();
    init_dmpc();
    init_timer();

}

HolohoverControlDMPCNode::~HolohoverControlDMPCNode()
{
    delete[] v_in;
    delete[] v_out;
    delete[] v_in_msg;
    delete[] v_in_msg_idx_first_received;
    delete[] v_out_msg;
    delete[] v_out_msg_idx_first_received;
    // delete[] v_in_publisher;
    // delete[] v_out_publisher;
    // delete[] v_in_subscriber;
    // delete[] v_out_subscriber;
    delete[] XV;
    delete[] v_in_msg_recv_buff;
    delete[] v_out_msg_recv_buff; 
}

void HolohoverControlDMPCNode::init_topics()
{
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControlStamped>(
            "drone/control",
            rclcpp::SensorDataQoS());

    // laopt_frequency_publisher = this->create_publisher<holohover_msgs::msg::HolohoverLaoptSpeedStamped>(
            // "control/laopt_speed",
            // rclcpp::SensorDataQoS());

    HolohoverTrajectory_publisher = this->create_publisher<holohover_msgs::msg::HolohoverTrajectory>(
            "control/HolohoverTrajectory",
            rclcpp::SensorDataQoS());

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverStateStamped>(
            "navigation/state", 10,
            std::bind(&HolohoverControlDMPCNode::state_callback, this, std::placeholders::_1));

    reference_subscription = this->create_subscription<holohover_msgs::msg::HolohoverState>(
            "control/state_ref", 10,
            std::bind(&HolohoverControlDMPCNode::ref_callback, this, std::placeholders::_1));
}

void HolohoverControlDMPCNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(control_settings.period),
            std::bind(&HolohoverControlDMPCNode::publish_control, this));
}

void HolohoverControlDMPCNode::publish_control()
{
    set_state_in_ocp();
    set_u_acc_curr_in_ocp();
    update_setpoint_in_ocp();

    holohover_msgs::msg::HolohoverControlStamped control_msg; //GS: move this to after solve?
    const steady_clock::time_point t_start = steady_clock::now();

    admm->solve(control_settings.maxiter,sol);

    const steady_clock::time_point t_end = steady_clock::now();
    const long duration_us = duration_cast<microseconds>(t_end - t_start).count();
    std::cout << "duration_ms  =" <<duration_us/1000 << std::endl;

    get_u_acc_from_sol();

    //publish_trajectory();

    convert_u_acc_to_u_signal();

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

void HolohoverControlDMPCNode::convert_u_acc_to_u_signal()
{
    motor_velocities = holohover.Ad_motor * motor_velocities + holohover.Bd_motor * last_control_signal;

    // calculate thrust bounds for next step
    Holohover::control_force_t<double> u_force_next_min, u_force_next_max;
    holohover.signal_to_thrust<double>(holohover.Ad_motor * motor_velocities + Holohover::control_force_t<double>::Constant(holohover.Bd_motor * holohover_props.idle_signal), u_force_next_min);
    holohover.signal_to_thrust<double>(holohover.Ad_motor * motor_velocities + Holohover::control_force_t<double>::Constant(holohover.Bd_motor * 1.0), u_force_next_max);

    // calculate next thrust and motor velocities
    Holohover::control_force_t<double> u_force_next;
    Holohover::state_t<double> state_next = holohover.Ad * state + holohover.Bd * u_acc_curr;
    holohover.control_acceleration_to_force(state_next, u_acc_next, u_force_next, u_force_next_min, u_force_next_max);    
    Holohover::control_force_t<double> motor_velocities_next;
    holohover.thrust_to_signal(u_force_next, motor_velocities_next);

    // calculate control from future motor velocities
    u_signal = (motor_velocities_next - holohover.Ad_motor * motor_velocities) / holohover.Bd_motor;

    // clip between 0 and 1
    u_signal = u_signal.cwiseMax(holohover_props.idle_signal).cwiseMin(1);

    // save control inputs for next iterations
    Holohover::control_force_t<double> u_force;
    holohover.signal_to_thrust(u_signal, u_force);
    holohover.control_force_to_acceleration(state, u_force, u_acc_curr);
    last_control_signal = u_signal;
}

void HolohoverControlDMPCNode::publish_trajectory( )
{

    holohover_msgs::msg::HolohoverTrajectory msg;
    msg.header.frame_id = "body";
    msg.header.stamp = this->now();

    //StateTrajectory = Eigen::Matrix<Scalar, ControlProblem::NX, N + 1>;
//    trajectory.FutureTrajectory = transcription.get_X_opt()[0][1];

    /*Eigen::MatrixXd X_opt = transcription.get_X_opt();
    msg.state_trajectory.resize(20);
    for (int i = 0; i < 20; i++)
    {
        msg.state_trajectory[i].x = X_opt(0,i);
        msg.state_trajectory[i].y = X_opt(1,i);
        msg.state_trajectory[i].v_x = X_opt(2,i);
        msg.state_trajectory[i].v_y = X_opt(3,i);
        msg.state_trajectory[i].yaw = X_opt(4,i);
        msg.state_trajectory[i].w_z = X_opt(5,i);
    }
    */

    HolohoverTrajectory_publisher->publish(msg);
}

void HolohoverControlDMPCNode::state_callback(const holohover_msgs::msg::HolohoverStateStamped &msg_state)
{
    state(0) = msg_state.state_msg.x;
    state(1) = msg_state.state_msg.y;
    state(2) = msg_state.state_msg.v_x;
    state(3) = msg_state.state_msg.v_y;
    state(4) = msg_state.state_msg.yaw;
    state(5) = msg_state.state_msg.w_z;
}

void HolohoverControlDMPCNode::ref_callback(const holohover_msgs::msg::HolohoverState &pose)
{
    ref = pose; //GS: what is this?
}

int main(int argc, char **argv) {
    std::string folder_name_sprob_ = "here"; 
    int my_id_ = 0;
    int Nagents_ = 0;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlDMPCNode>(folder_name_sprob_, my_id_, Nagents_));
    rclcpp::shutdown();
    return 0;
}


//GS BEGIN

void HolohoverControlDMPCNode::set_state_in_ocp()
{
    p.segment(0,control_settings.nx) = state;
    admm->lbA.block(control_settings.idx_eqx0,0,control_settings.nx,1) = state;
    admm->ubA.block(control_settings.idx_eqx0,0,control_settings.nx,1) = state;
}

void HolohoverControlDMPCNode::set_u_acc_curr_in_ocp()
{
    p.segment(control_settings.nx,control_settings.nu) = u_acc_curr;
    admm->lbA.block(control_settings.idx_equ0,0,control_settings.nu,1) = u_acc_curr;
    admm->ubA.block(control_settings.idx_equ0,0,control_settings.nu,1) = u_acc_curr;
}

void HolohoverControlDMPCNode::get_u_acc_from_sol()
{
    //u_acc_curr = sol.block(control_settings.idx_u0,0,control_settings.nu,1); //updated in convert_u_acc_to_u_signal
    u_acc_next = sol.block(control_settings.idx_u1,0,control_settings.nu,1);
}

void HolohoverControlDMPCNode::update_setpoint_in_ocp(){

    p.segment(0,control_settings.nx) = state;                           //GS: remove, because we have set_state_in_ocp?
    p.segment(control_settings.nx,control_settings.nu) = u_acc_curr;    //GS: remove, because we have set_u_acc_curr_in_ocp?
    p.segment(control_settings.nx+control_settings.nu,control_settings.nxd) = state_ref;

    std::string function_library = folder_name_sprob + "/locFuns.so";

    casadi::DM z_cas;
    casadi::DM p_cas;

    // Use CasADi's "external" to load the compiled function
    casadi::Function f; // = casadi::external("gradFun1","sProb_chain/locFuns.so");
    std::string str;

    std::vector<casadi::DM> arg; // = {z};
    std::vector<casadi::DM> res; // = f(arg);

    int nx_ = admm->g.size();


    VectorXd z = Eigen::VectorXd::Zero(nx_);
  
    z_cas = Eigen2casadi(z);
    p_cas = Eigen2casadi(p);

    //g
    str = "gradFun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,function_library);
    arg = {z_cas,p_cas};
    res = f(arg);
    admm->sprob.g[my_id]= casadi2EigenVector(res[0]);
    
    //beq
    str = "eqfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,function_library);
    // arg = {z_cas};
    res = f(arg);
    int ng_ = res[0].size1();
    int cols_ = res[0].size2();
    admm->sprob.beq[my_id] = Eigen::VectorXd::Zero(ng_);        
    std::memcpy(admm->sprob.beq[my_id].data(), res.at(0).ptr(), sizeof(double)*ng_*cols_);
    admm->sprob.beq[my_id] = -admm->sprob.beq[my_id];

    admm->update_g_beq();
   
}

void HolohoverControlDMPCNode::init_dmpc()
{
   //todo 
}



// Dense to Dense
Eigen::MatrixXd HolohoverControlDMPCNode::casadi2Eigen ( const casadi::DM& A ){
    // This method is based on code by Petr Listov, see https://groups.google.com/g/casadi-users/c/npPcKItdLN8
    
    casadi::Sparsity SpA = A.get_sparsity();
    std::vector<long long int> output_row, output_col;
    SpA.get_triplet(output_row, output_col);
    std::vector<double> values = A.get_nonzeros();
    using T = Eigen::Triplet<double>;
    std::vector<T> TripletList;
    TripletList.resize(values.size());
    for(int k = 0; k < values.size(); ++k){
        if (values[k] == casadi::inf){
            values[k] = DBL_MAX;
        }
        else if (values[k] == -casadi::inf){
            values[k] = -DBL_MAX;
        }
        TripletList[k] = T(output_row[k], output_col[k], values[k]);
    }
    Eigen::SparseMatrix<double> SpMatrx(A.size1(), A.size2());
    SpMatrx.setFromTriplets(TripletList.begin(), TripletList.end());

    Eigen::MatrixXd DeMatrx = SpMatrx.toDense();
    return DeMatrx;
}

// Dense to Dense
Eigen::VectorXd HolohoverControlDMPCNode::casadi2EigenVector ( const casadi::DM& A ){
    // This method is based on code by Petr Listov, see https://groups.google.com/g/casadi-users/c/npPcKItdLN8
    
    casadi::Sparsity SpA = A.get_sparsity();
    std::vector<long long int> output_row, output_col;
    SpA.get_triplet(output_row, output_col);
    std::vector<double> values = A.get_nonzeros();
    using T = Eigen::Triplet<double>;
    std::vector<T> TripletList;
    TripletList.resize(values.size());
    for(int k = 0; k < values.size(); ++k){
        if (values[k] == casadi::inf){
            values[k] = DBL_MAX;
        }
        else if (values[k] == -casadi::inf){
            values[k] = -DBL_MAX;
        }
        TripletList[k] = T(output_row[k], output_col[k], values[k]);
    }
    Eigen::VectorXd DeVec;
    if (A.size2() == 1){ // column-vector
        DeVec = Eigen::VectorXd::Zero(A.size1());
        
        for (int k = 0; k < values.size(); k++){
            DeVec[output_row[k]] = values[k];
        }
    } else { //row-vector (will be transposed)
        DeVec = Eigen::VectorXd::Zero(A.size2());
        for (int k = 0; k < values.size(); k++){
            DeVec[output_col[k]] = values[k];
        }
    }

    for (int k = 0; k < DeVec.size(); k++){
         if (DeVec[k] == casadi::inf){
            DeVec[k] = DBL_MAX;
        }
    }

    return DeVec;
}

casadi::DM HolohoverControlDMPCNode::Eigen2casadi( const Eigen::VectorXd& in){
    int length = in.size();
    casadi::DM out = casadi::DM::zeros(length,1);
    std::memcpy(out.ptr(), in.data(), sizeof(double)*length*1);
    return out;
}

//GS END
