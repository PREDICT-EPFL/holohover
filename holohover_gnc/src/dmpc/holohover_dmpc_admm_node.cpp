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

#include "holohover_dmpc_admm_node.hpp"

using namespace std::chrono;

// std::string HolohoverDmpcAdmmNode::build_node_name(int my_id_){
//     std::string out = "admm_dmpc_node" + std::to_string(my_id_);
//     return out;
// } 

HolohoverDmpcAdmmNode::HolohoverDmpcAdmmNode() :
        Node("dmpc_node"),
        holohover_props(load_holohover_pros(*this)),
        control_settings(load_control_dmpc_settings(*this)),
        holohover(holohover_props, control_settings.period),
        sprob(control_settings.Nagents)
{
    my_id = control_settings.my_id;
    Nagents = control_settings.Nagents;    

    // init state
    state.setZero();
    u_acc_curr.setZero();
    u_acc_next.setZero();
    motor_velocities.setZero();
    last_control_signal.setZero();
    u_signal.setZero();


    p = Eigen::VectorXd::Zero(control_settings.nx+control_settings.nu+control_settings.nxd);
    state_ref = p.segment(control_settings.nx+control_settings.nu,control_settings.nxd); //todo


    build_qp();

    nx = sprob.H[my_id].rows();
    ng = sprob.Aeq[my_id].rows();
    nh = sprob.Aineq[my_id].rows();

    std::cout << "nx = " << nx << " , " << "ng = " << ng << " , " << "nh = " << nh << std::endl;

    Ncons = sprob.A[my_id].rows();

    double rho = 1.0;
    H_bar = sprob.H[my_id] + rho * MatrixXd::Identity(nx,nx);
    g = sprob.g[my_id];
    A = MatrixXd::Zero(ng+nh,nx);
    A.block(0,0,ng,nx) = sprob.Aeq[my_id];
    
    if (nh > 0){
        A.block(ng,0,nh,nx) = sprob.Aineq[my_id];
    }

    lbA = VectorXd::Zero(ng+nh);
    ubA = VectorXd::Zero(ng+nh);
    lbA.block(0,0,ng,1) = sprob.beq[my_id];
    ubA.block(0,0,ng,1) = sprob.beq[my_id];

    if (nh > 0){
        lbA.block(ng,0,nh,1).setConstant(-pow(10,20));  
        ubA.block(ng,0,nh,1) = sprob.bineq[my_id];
    }

    z       = VectorXd::Zero(nx);
    zbar   = VectorXd::Zero(nx);
    gam     = VectorXd::Zero(nx);
    ub = sprob.ub[my_id];
    lb = sprob.lb[my_id];

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

    // XV = new std::vector<double>[N_og];
    for (int i = 0; i < N_og; i++){
        XV.push_back(std::vector<double>());
        auto idx = og_idx_to_idx.find(i);
        XV[i].resize(numCopies(idx->second)+1); //XV stores local original variable and copies from out-neighbors
    }

    //initialize comms
    // v_out
    receive_vout_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    receive_vout_options.callback_group = receive_vout_cb_group;
    v_out_msg.resize(N_out_neighbors); // = new holohover_msgs::msg::HolohoverADMMStamped[N_out_neighbors];
    v_out_subscriber.resize(N_out_neighbors); // = new rclcpp::Subscription<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr[N_out_neighbors];
    v_out_publisher.resize(N_out_neighbors); // = new rclcpp::Publisher<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr[N_out_neighbors];
    bound_received_vout_callback.resize(N_out_neighbors);
    for (int i = 0; i < N_out_neighbors; i++){
        int neighbor_id = v_out[i].copying_agent;       

        v_out_msg[i].val_length = v_out[i].nv;
        v_out_msg[i].idx_length = v_out[i].nv;
        v_out_msg[i].value.resize(v_out_msg[i].val_length);
        v_out_msg[i].idx.resize(v_out_msg[i].idx_length);
        v_out_msg[i].seq_number = 0;
        for (int idx_row = 0; idx_row<v_out_msg[i].val_length; idx_row++) {
            v_out_msg[i].value[idx_row] = std::numeric_limits<double>::quiet_NaN();
        }
        v_out_msg[i].id_sender = my_id;

        std::string v_out_pub_topic = "/agent";
        v_out_pub_topic.append(std::to_string(my_id));
        v_out_pub_topic.append("ogforagent");
        v_out_pub_topic.append(std::to_string(neighbor_id));
        v_out_publisher[i] = this->create_publisher<holohover_msgs::msg::HolohoverADMMStamped>(
            v_out_pub_topic,
            rclcpp::SystemDefaultsQoS());
        std::cout << "I am agent " << my_id << " and I am publishing to " << v_out_pub_topic << std::endl;  
    
        std::string v_out_sub_topic = "/agent";
        v_out_sub_topic.append(std::to_string(neighbor_id));
        v_out_sub_topic.append("copyofagent");
        v_out_sub_topic.append(std::to_string(my_id));
        bound_received_vout_callback[i] = std::bind(&HolohoverDmpcAdmmNode::received_vout_callback, this, std::placeholders::_1, i); 
        v_out_subscriber[i] = this->create_subscription<holohover_msgs::msg::HolohoverADMMStamped>(
                            v_out_sub_topic, rclcpp::SystemDefaultsQoS(),
                            bound_received_vout_callback[i],receive_vout_options); //i
        // v_out_subscriber[i] = this->create_subscription<holohover_msgs::msg::HolohoverADMMStamped>(
        //                     v_out_sub_topic, 10,
        //                     std::bind(&HolohoverDmpcAdmmNode::received_vout_callback, this, std::placeholders::_1)); //i
        std::cout << "I am agent " << my_id << " and I am subscribing to " << v_out_sub_topic << std::endl;
    }

    // v_in
    receive_vin_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    receive_vin_options.callback_group = receive_vin_cb_group;
    v_in_msg.resize(N_in_neighbors); // = new holohover_msgs::msg::HolohoverADMMStamped[N_in_neighbors];
    v_in_subscriber.resize(N_in_neighbors); // = new rclcpp::Subscription<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr[N_in_neighbors];
    v_in_publisher.resize(N_in_neighbors); // = new rclcpp::Publisher<holohover_msgs::msg::HolohoverADMMStamped>::SharedPtr[N_in_neighbors];
    bound_received_vin_callback.resize(N_in_neighbors);
    for (int i = 0; i < N_in_neighbors; i++){
        int neighbor_id = v_in[i].original_agent;

        v_in_msg[i].val_length = v_in[i].nv;
        v_in_msg[i].value.resize(v_in_msg[i].val_length);
        v_in_msg[i].idx_length = v_in[i].nv;
        v_in_msg[i].idx.resize(v_in_msg[i].idx_length);
        Eigen::VectorXi::Map(&v_in_msg[i].idx[0], v_in[i].og_idx.size()) = v_in[i].og_idx;
        v_in_msg[i].seq_number = 0;
        for (int idx_row = 0; idx_row<v_in_msg[i].val_length; idx_row++) {
            v_in_msg[i].value[idx_row] = std::numeric_limits<double>::quiet_NaN();
        }
        v_in_msg[i].id_sender = my_id;

        std::string v_in_pub_topic = "/agent";
        v_in_pub_topic.append(std::to_string(my_id));
        v_in_pub_topic.append("copyofagent");
        v_in_pub_topic.append(std::to_string(neighbor_id));
        v_in_publisher[i] = this->create_publisher<holohover_msgs::msg::HolohoverADMMStamped>(
            v_in_pub_topic,
            rclcpp::SystemDefaultsQoS());
        std::cout << "I am agent " << my_id << " and I am publishing to " << v_in_pub_topic << std::endl;

        std::string v_in_sub_topic = "/agent";
        v_in_sub_topic.append(std::to_string(neighbor_id));
        v_in_sub_topic.append("ogforagent");
        v_in_sub_topic.append(std::to_string(my_id));
        bound_received_vin_callback[i] = std::bind(&HolohoverDmpcAdmmNode::received_vin_callback, this, std::placeholders::_1, i);
        v_in_subscriber[i] = this->create_subscription<holohover_msgs::msg::HolohoverADMMStamped>(
                            v_in_sub_topic, rclcpp::SystemDefaultsQoS(),
                            bound_received_vin_callback[i],receive_vin_options);  //i  
        // v_in_subscriber[i] = this->create_subscription<holohover_msgs::msg::HolohoverADMMStamped>(
                            // v_in_sub_topic, rclcpp::SystemDefaultsQoS,
                            // std::bind(&HolohoverDmpcAdmmNode::received_vin_callback, this, std::placeholders::_1));  //i 
        std::cout << "I am agent " << my_id << " and I am subscribing to " << v_in_sub_topic << std::endl;
    }
       
    // v_in_msg_idx_first_received.resize(N_in_neighbors);
    // v_out_msg_idx_first_received.resize(N_out_neighbors);

    
    // std::cout << "calling init_dmpc" << std::endl;
    // init_dmpc();
    std::cout << "calling init_timer" << std::endl;
    init_timer();
    std::cout << "calling init_topics" << std::endl;
    init_topics();

    std::cout << "finished constructing dmpc admm node" << std::endl;

}

HolohoverDmpcAdmmNode::~HolohoverDmpcAdmmNode()
{

}

void HolohoverDmpcAdmmNode::init_topics()
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
            std::bind(&HolohoverDmpcAdmmNode::state_callback, this, std::placeholders::_1));

    reference_subscription = this->create_subscription<holohover_msgs::msg::HolohoverState>(
            "control/state_ref", 10,
            std::bind(&HolohoverDmpcAdmmNode::ref_callback, this, std::placeholders::_1));

    publish_control_subscription = this->create_subscription<std_msgs::msg::UInt64>(
            "publish_control", 10,
            std::bind(&HolohoverDmpcAdmmNode::publish_control, this, std::placeholders::_1));
}

void HolohoverDmpcAdmmNode::init_timer()
{
    // timer = this->create_wall_timer(
    //         std::chrono::duration<double>(control_settings.period),
    //         std::bind(&HolohoverDmpcAdmmNode::publish_control, this));
}

void HolohoverDmpcAdmmNode::init_comms(){
    //call this before you call solve()
    //send around the indices of v_inMsg and v_outMsg one. Then set idx_length to zero for future messages.

    //send vin
    for (int i = 0; i < N_in_neighbors; i++){
        v_in_msg[i].seq_number += 1;
        v_in_msg[i].header.frame_id = "body"; 
        v_in_msg[i].header.stamp = this->now(); 
        Eigen::VectorXd::Map(&v_in_msg[i].value[0], v_in[i].val.size()) = v_in[i].val;
        std::cout << "sending to in-neighbor " << i << std::endl; 
        v_in_publisher[i]->publish(v_in_msg[i]); //ros
    }

    //receive vout

    Eigen::Array<bool,Dynamic,1> received(N_out_neighbors,1);
    received.fill(false);
    while (!received.all()){
        for (int i = 0; i < N_out_neighbors; i++){
            if (!received(i)){
                if (received_vout[i]){ //has received a new message
                    received[i] = true;
                    received_vout[i] = false;
                    std::cout << "have received from out-neighbor " << i << std::endl;
                    v_out_msg_idx_first_received[i] = v_out_msg_recv_buff[i].idx;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        for (int i = 0; i < N_in_neighbors; i++){
            // std::cout << "resending to in-neighbor " << i << std::endl; 
            v_in_publisher[i]->publish(v_in_msg[i]); //ros
        }
        for (int i = 0; i < N_out_neighbors; i++){
            v_out_publisher[i]->publish(v_out_msg[i]); //ros
        }
    }

    //send vout
    for (int i = 0; i < N_out_neighbors; i++){
        v_out_msg[i].seq_number += 1;
        v_out_msg[i].header.frame_id = "body"; 
        v_out_msg[i].header.stamp = this->now();

        int idx_row = 0;
        for (int j = 0; j < nx; j++){
            if (isOriginal[j]){
                v_out_msg[i].value[idx_row] = zbar[j];
                idx_row += 1;
            }
            if (idx_row == v_out_msg[i].val_length){
                break;        
            }
        }
        v_out_publisher[i]->publish(v_out_msg[i]);
    }

    //receive vin
    received = Eigen::Array<bool,Dynamic,1>(N_in_neighbors,1);
    received.fill(false);
    while (!received.all()){
        for (int i = 0; i < N_in_neighbors; i++){
            if (!received(i)){
                if (received_vin[i]){
                    received_vin[i] = false;
                    received[i] = true;
                    v_in_msg_idx_first_received[i] = v_in_msg_recv_buff[i].idx;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        for (int i = 0; i < N_in_neighbors; i++){
            v_in_publisher[i]->publish(v_in_msg[i]); //ros
        }
        for (int i = 0; i < N_out_neighbors; i++){
            v_out_publisher[i]->publish(v_out_msg[i]); //ros
        }
    }


    for (int i = 0; i < N_out_neighbors; i++){
        v_out_msg[i].idx_length = 0;
        v_out_msg[i].idx.resize(0);        
    }
    for (int i = 0; i < N_in_neighbors; i++){
        v_in_msg[i].idx_length = 0;
        v_in_msg[i].idx.resize(0);
    }


    

    return;
}

void HolohoverDmpcAdmmNode::publish_control(const std_msgs::msg::UInt64 &publish_control_msg)
{
    std::cout << "starting publish_control()" << std::endl;
    set_state_in_ocp();
    set_u_acc_curr_in_ocp();
    update_setpoint_in_ocp();

    holohover_msgs::msg::HolohoverControlStamped control_msg; //GS: move this to after solve?
    const steady_clock::time_point t_start = steady_clock::now();

    std::cout << "calling solve" << std::endl;
    solve(control_settings.maxiter);

    const steady_clock::time_point t_end = steady_clock::now();
    const long duration_us = duration_cast<microseconds>(t_end - t_start).count();
    std::cout << "ADMM duration_ms  =" <<duration_us/1000 << std::endl;

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

void HolohoverDmpcAdmmNode::convert_u_acc_to_u_signal()
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

void HolohoverDmpcAdmmNode::publish_trajectory( )
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

void HolohoverDmpcAdmmNode::state_callback(const holohover_msgs::msg::HolohoverStateStamped &msg_state)
{
    state(0) = msg_state.state_msg.x;
    state(1) = msg_state.state_msg.y;
    state(2) = msg_state.state_msg.v_x;
    state(3) = msg_state.state_msg.v_y;
    state(4) = msg_state.state_msg.yaw;
    state(5) = msg_state.state_msg.w_z;
}

void HolohoverDmpcAdmmNode::ref_callback(const holohover_msgs::msg::HolohoverState &pose)
{
    ref = pose; //GS: what is this?
}




//GS BEGIN

void HolohoverDmpcAdmmNode::set_state_in_ocp()
{
    p.segment(0,control_settings.nx) = state;
    lbA.block(control_settings.idx_eqx0,0,control_settings.nx,1) = state;
    ubA.block(control_settings.idx_eqx0,0,control_settings.nx,1) = state;
}

void HolohoverDmpcAdmmNode::set_u_acc_curr_in_ocp()
{
    p.segment(control_settings.nx,control_settings.nu) = u_acc_curr;
    lbA.block(control_settings.idx_equ0,0,control_settings.nu,1) = u_acc_curr;
    ubA.block(control_settings.idx_equ0,0,control_settings.nu,1) = u_acc_curr;
}

void HolohoverDmpcAdmmNode::get_u_acc_from_sol()
{
    //u_acc_curr = sol.block(control_settings.idx_u0,0,control_settings.nu,1); //updated in convert_u_acc_to_u_signal
    u_acc_next = zbar.block(control_settings.idx_u1,0,control_settings.nu,1);
}

void HolohoverDmpcAdmmNode::update_setpoint_in_ocp(){

    p.segment(0,control_settings.nx) = state;                           //GS: remove, because we have set_state_in_ocp?
    p.segment(control_settings.nx,control_settings.nu) = u_acc_curr;    //GS: remove, because we have set_u_acc_curr_in_ocp?
    p.segment(control_settings.nx+control_settings.nu,control_settings.nxd) = state_ref;

    std::string function_library = control_settings.folder_name_sprob + "/locFuns.so";

    casadi::DM z_cas;
    casadi::DM p_cas;

    // Use CasADi's "external" to load the compiled function
    casadi::Function f; // = casadi::external("gradFun1","sProb_chain/locFuns.so");
    std::string str;

    std::vector<casadi::DM> arg; // = {z};
    std::vector<casadi::DM> res; // = f(arg);

    int nx_ = g.size();


    VectorXd z = Eigen::VectorXd::Zero(nx_);
  
    z_cas = Eigen2casadi(z);
    p_cas = Eigen2casadi(p);

    //g
    str = "gradFun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,function_library);
    arg = {z_cas,p_cas};
    res = f(arg);
    sprob.g[my_id]= casadi2EigenVector(res[0]);
    
    //beq
    str = "eqfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,function_library);
    // arg = {z_cas};
    res = f(arg);
    int ng_ = res[0].size1();
    int cols_ = res[0].size2();
    sprob.beq[my_id] = Eigen::VectorXd::Zero(ng_);        
    std::memcpy(sprob.beq[my_id].data(), res.at(0).ptr(), sizeof(double)*ng_*cols_);
    sprob.beq[my_id] = -sprob.beq[my_id];

    update_g_beq();
   
}

void HolohoverDmpcAdmmNode::init_dmpc()
{
    init_comms();
    std::cout << "comms initialized" << std::endl;
    z     = VectorXd::Zero(nx);
    zbar  = VectorXd::Zero(nx);
    gam   = VectorXd::Zero(nx);

    solve(10); //initializes data structures in admmAgent

    //reset all ADMM variables to zero. 
    z     = VectorXd::Zero(nx);
    zbar  = VectorXd::Zero(nx);
    gam   = VectorXd::Zero(nx);
    return;
}



// Dense to Dense
Eigen::MatrixXd HolohoverDmpcAdmmNode::casadi2Eigen ( const casadi::DM& A ){
    // This method is based on code by Petr Listov, see https://groups.google.com/g/casadi-users/c/npPcKItdLN8
    
    casadi::Sparsity SpA = A.get_sparsity();
    std::vector<long long int> output_row, output_col;
    SpA.get_triplet(output_row, output_col);
    std::vector<double> values = A.get_nonzeros();
    using T = Eigen::Triplet<double>;
    std::vector<T> TripletList;
    TripletList.resize(values.size());
    for(unsigned int k = 0; k < values.size(); ++k){
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
Eigen::VectorXd HolohoverDmpcAdmmNode::casadi2EigenVector ( const casadi::DM& A ){
    // This method is based on code by Petr Listov, see https://groups.google.com/g/casadi-users/c/npPcKItdLN8
    
    casadi::Sparsity SpA = A.get_sparsity();
    std::vector<long long int> output_row, output_col;
    SpA.get_triplet(output_row, output_col);
    std::vector<double> values = A.get_nonzeros();
    using T = Eigen::Triplet<double>;
    std::vector<T> TripletList;
    TripletList.resize(values.size());
    for(unsigned int k = 0; k < values.size(); ++k){
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
        
        for (unsigned int k = 0; k < values.size(); k++){
            DeVec[output_row[k]] = values[k];
        }
    } else { //row-vector (will be transposed)
        DeVec = Eigen::VectorXd::Zero(A.size2());
        for (unsigned int k = 0; k < values.size(); k++){
            DeVec[output_col[k]] = values[k];
        }
    }

    for (unsigned int k = 0; k < DeVec.size(); k++){
         if (DeVec[k] == casadi::inf){
            DeVec[k] = DBL_MAX;
        }
    }

    return DeVec;
}

casadi::DM HolohoverDmpcAdmmNode::Eigen2casadi( const Eigen::VectorXd& in){
    int length = in.size();
    casadi::DM out = casadi::DM::zeros(length,1);
    std::memcpy(out.ptr(), in.data(), sizeof(double)*length*1);
    return out;
}

//GS END

void HolohoverDmpcAdmmNode::build_qp()
{
    sprob.read_AA(control_settings.folder_name_sprob,Nagents);
    sprob.read_ublb(control_settings.folder_name_sprob,my_id);
    std::string functionLibrary = control_settings.folder_name_sprob + "/locFuns.so";

    casadi::DM z_cas;
    casadi::DM p_cas;

    // Use CasADi's "external" to load the compiled function
    casadi::Function f; // = casadi::external("gradFun1","sProb_chain/locFuns.so");
    std::string str;

    std::vector<casadi::DM> arg; // = {z};
    std::vector<casadi::DM> res; // = f(arg);

    int nx_ = sprob.A[my_id].cols();


    VectorXd z_ = Eigen::VectorXd::Zero(nx_);
  
    z_cas = Eigen2casadi(z_);
    p_cas = Eigen2casadi(p);

    //H
    str = "HessFfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    arg = {z_cas,p_cas};
    res = f(arg);
    casadi::DM H = res[0];
    MatrixXd Heig = casadi2Eigen(H);
    sprob.H[my_id] = Heig;

    //g
    str = "gradFun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    res = f(arg);
    sprob.g[my_id] = Eigen::VectorXd::Zero(nx_);
    sprob.g[my_id] = casadi2EigenVector(res[0]);

    //Aeq
    str = "JGfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    res = f(arg);
    casadi::DM Aeq = res[0];
    sprob.Aeq[my_id] = casadi2Eigen(Aeq);

    //Aineq
    str = "JHfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    res = f(arg);
    casadi::DM Aineq = res[0];
    sprob.Aineq[my_id] = casadi2Eigen(Aineq);

    //beq
    str = "eqfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    res = f(arg);
    int ng_ = res[0].size1();
    int cols_ = res[0].size2();
    sprob.beq[my_id] = Eigen::VectorXd::Zero(ng_);        
    std::memcpy(sprob.beq[my_id].data(), res.at(0).ptr(), sizeof(double)*ng_*cols_);
    sprob.beq[my_id] = -sprob.beq[my_id];

    //bineq
    str = "ineqfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    res = f(arg);
    int nh_ = res[0].size1();
    cols_ = res[0].size2();
    sprob.bineq[my_id] = Eigen::VectorXd::Zero(nh_);        
    std::memcpy(sprob.bineq[my_id].data(), res.at(0).ptr(), sizeof(double)*nh_*cols_);
    sprob.bineq[my_id] = -sprob.bineq[my_id];

    VectorXd mlb = sprob.lb[my_id];

    for (int k = 0; k < sprob.lb[my_id].size(); k++){
        if (sprob.lb[my_id][k] == -casadi::inf){
            sprob.lb[my_id][k] = -pow(10,20); 
        } else if (sprob.lb[my_id][k] == casadi::inf){
            sprob.lb[my_id][k] = pow(10,20);
        }
    }

    for (int k = 0; k < sprob.ub[my_id].size(); k++){
        if (sprob.ub[my_id][k] == -casadi::inf){
            sprob.ub[my_id][k] = -pow(10,20);
        } else if (sprob.ub[my_id][k] == casadi::inf){
            sprob.ub[my_id][k] = pow(10,20);
        }
    }


}

void HolohoverDmpcAdmmNode::init_coupling()
{
    //Coupling information
    Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Coup = -MatrixXi::Ones(nx,2); //col1: owning agent ID; col2: index in nx of owning agent

    std::vector<int> nnx (Nagents);
    for (int i = 0; i < Nagents; i++){
        nnx[i] = sprob.A[i].cols();
    }
    MatrixXd tmp;
    std::vector<int>::iterator it;
    VectorXd nv_out = VectorXd::Zero(Nagents);

    //Identify coupling structure
    //Out-neighbors
    N_og = 0;
    for (int i = 0; i < nx; i++){
        for (int j = 0; j < Ncons; j++){
            if (sprob.A[my_id](j,i) >= 0.9){
                if (!isOriginal[i]){
                    isOriginal[i] = 1;
                    og_idx_to_idx.insert({N_og,i});
                    idx_to_og_idx.insert({i,N_og});
                    N_og += 1;
                    Coup(i,0) = my_id;
                    Coup(i,1) = i;
                }                
                bool isDestination = 0;
                for (int k = 0; k < Nagents; k++){
                    tmp = MatrixXd::Zero(1,nnx[k]);
                    tmp = sprob.A[k].block(j,0,1,nnx[k]);
                    isDestination = (tmp.array() <= -0.9).any();
                    if (isDestination){
                        it = std::find(out_neighbors.begin(), out_neighbors.end(),k);
                        nv_out[k] += 1;
                        if( it == out_neighbors.end()) {
                            out_neighbors.push_back(k);
                        }
                        break;
                    }                 
                }
            }
        }        
    }
    N_out_neighbors = out_neighbors.size();

    //In-neighbors
    bool isSource;
    VectorXd nv_in = VectorXd::Zero(Nagents);
    for (int i = 0; i < nx; i++){
        isSource = 0;
        for (int j = 0; j < Ncons; j++){
            if(sprob.A[my_id](j,i) <= -0.9){
                isCopy(i) = true;
                for (int k = 0; k < Nagents; k++){
                    tmp.resize(1,nnx[k]);
                    tmp = sprob.A[k].block(j,0,1,nnx[k]);
                    isSource = (tmp.array() >= 0.9).any();
                    if (isSource){
                        isSource = 0;
                        Coup(i,0) = k;
                        for (int l = 0; l < sprob.A[k].cols(); l++){
                            if (sprob.A[k](j,l) == 1){
                                Coup(i,1) = l;
                                break;
                            }
                        }
                        nv_in[k] += 1;
                        it = std::find(in_neighbors.begin(), in_neighbors.end(),k);
                        if( it == in_neighbors.end()) {
                            in_neighbors.push_back(k);
                        }
                        break;
                    }
                }
                break;
            }
        }
    }
    N_in_neighbors = in_neighbors.size();
    isCopy.resize(nx);

    for (int i = 0; i < nx; i++){
        if (!isOriginal(i) && !isCopy(i)){
            isOriginal(i) = true;
            og_idx_to_idx.insert({N_og,i});
            idx_to_og_idx.insert({i,N_og});
            Coup(i,0) = my_id;
            Coup(i,1) = i;
            N_og += 1;
        }
    }


    //Setup v_in and v_out
    // v_in = new vCpy[N_in_neighbors];
    // v_out = new vCpy[N_out_neighbors];

    //In-neighbors
    int nij = 0;
    for (int j = 0; j < N_in_neighbors; j++){
        v_in.push_back(vCpy());
        nij = 0;
        for (int i = 0; i < nx; i++){
            if (Coup(i,0) == in_neighbors[j]){ //Variable i was copied from agent j
                nij+=1;
            }
        }
        v_in[j].original_agent = in_neighbors[j];
        v_in[j].copying_agent = my_id;
        v_in[j].val = VectorXd::Zero(nij);
        v_in[j].cpy_idx = VectorXi::Zero(nij);
        v_in[j].og_idx = VectorXi::Zero(nij);
        v_in[j].nv = nij;
        nij = 0;
        for (int i = 0; i < nx; i++){
            if (Coup(i,0) == in_neighbors[j]){ //Variable i was copied from agent j
                v_in[j].cpy_idx(nij) = i;
                v_in[j].og_idx(nij) = Coup(i,1);
                v_in[j].og_idx_to_cpy_idx.insert({v_in[j].og_idx(nij),v_in[j].cpy_idx(nij)});
                nij+=1;
            }
        }
    }

    //Out-neighbors
    for (int j = 0; j < N_out_neighbors; j++){
        v_out.push_back(vCpy());
        v_out[j].copying_agent = out_neighbors[j];
        v_out[j].original_agent = my_id;
        v_out[j].nv = nv_out[out_neighbors[j]];
        v_out[j].val = VectorXd::Zero(v_out[j].nv);
        v_out[j].cpy_idx = VectorXi::Zero(v_out[j].nv);
        v_out[j].og_idx = VectorXi::Zero(v_out[j].nv);
        int k = out_neighbors[j];

        int nij = 0;
        for (int i = 0; i < nx; i++){
            bool isDestination = 0;
            for (int c = 0; c < Ncons; c++){
                if (sprob.A[my_id](c,i) == 1){
                    tmp.resize(1,nnx[k]);
                    tmp = sprob.A[k].block(c,0,1,nnx[k]);
                    isDestination = (tmp.array() == -1.0).any();
                    if (isDestination){
                        for (int l = 0; l < nnx[k]; l++){
                            if (tmp(0,l) == -1){
                                v_out[j].cpy_idx[nij] = l;
                                v_out[j].og_idx[nij] = i;
                                numCopies[i] += 0;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    received_vin  = Eigen::Array<bool,Dynamic,1>(N_in_neighbors,1);
    received_vout = Eigen::Array<bool,Dynamic,1>(N_out_neighbors,1);
    received_vin.fill(false);
    received_vout.fill(false);
    
    v_in_msg_recv_buff.resize(N_in_neighbors); //= new holohover_msgs::msg::HolohoverADMMStamped[N_in_neighbors]; 
    v_out_msg_recv_buff.resize(N_out_neighbors); //= new holohover_msgs::msg::HolohoverADMMStamped[N_in_neighbors]; 

    return;
}

int HolohoverDmpcAdmmNode::solve(unsigned int maxiter_)
{
    // zbar = zbar_;
    for (unsigned int iter = 0; iter < maxiter_; iter++){

        // std::cout << "Starting ADMM iteration " << iter << std::endl;

        //Step 1: local z update

        // std::cout << "updating g_bar" << std::endl;
        g_bar = g + gam - rho*zbar;
        nWSR = 10000;
        // std::cout << "calling qpOASES" << std::endl;
        loc_prob.hotstart(g_bar.data(),
                           lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR);
        // std::cout << "extract qpOASES solution" << std::endl;
        loc_prob.getPrimalSolution(z.data());
        // std::cout << "put qpOASES solution into XV" << std::endl;
        for (int i = 0; i < N_og; i++){
            auto idx = og_idx_to_idx.find(i);
            XV[i].clear();
            XV[i].push_back(z(idx->second));
        }
        
        // std::cout << "start the averaging" << std::endl;
        //Communication
        update_v_in();
        send_vin_receive_vout();

        //Step 2: averaging
        double avg;
        for (int i = 0; i < N_og; i++){
            avg = std::accumulate(XV[i].begin(), XV[i].end(), 0.0) / XV[i].size();
            auto idx = og_idx_to_idx.find(i);
            zbar[idx->second] = avg;
        }

        //Communication
        update_v_out();
        send_vout_receive_vin();

        // Step 3: dual update
        // std::cout << "dual update" << std::endl;
        gam = gam + rho*(z-zbar);  

    }

    // zbar_ = zbar; //update output variable
    
    return 0;
}

void HolohoverDmpcAdmmNode::received_vin_callback(const holohover_msgs::msg::HolohoverADMMStamped &v_in_msg_, int in_neighbor_idx_){ //
    // int in_neighbor_idx_ = 0;
    if(!received_vin(in_neighbor_idx_)){     
        // std::cout << "I am agent " << my_id << " and I invoked vin callback for in neighbor " << in_neighbor_idx_ << std::endl;
        v_in_msg_recv_buff[in_neighbor_idx_] = v_in_msg_;
        received_vin(in_neighbor_idx_) = true;
    }
}

void HolohoverDmpcAdmmNode::received_vout_callback(const holohover_msgs::msg::HolohoverADMMStamped &v_out_msg_, int out_neighbor_idx_){ //
    // int out_neighbor_idx_ = 0;
    if(!received_vout(out_neighbor_idx_)){    
        // std::cout << "I am agent "<< my_id << " and I invoked vout callback for out neighbor " << out_neighbor_idx_ << std::endl;
        v_out_msg_recv_buff[out_neighbor_idx_] = v_out_msg_;
        received_vout(out_neighbor_idx_) = true; 
    }  
} 

void HolohoverDmpcAdmmNode::send_vin_receive_vout(){
    //send copies in vin and receive copies in vout
    //this is done before the averaging step

    //send vin
    //send_vin_timer.tic();
    for (int i = 0; i < N_in_neighbors; i++){
        v_in_msg[i].seq_number += 1;
        v_in_msg[i].header.frame_id = "body"; 
        v_in_msg[i].header.stamp = this->now(); 
        Eigen::VectorXd::Map(&v_in_msg[i].value[0], v_in[i].val.size()) = v_in[i].val;
        v_in_publisher[i]->publish(v_in_msg[i]); //ros
    }
    //send_vin_timer.toc();
    //receive vout
    //receive_vout_timer.tic();
    Eigen::Array<bool,Dynamic,1> received(N_out_neighbors,1);
    received.fill(false);

    while (!received.all()){
        for (int i = 0; i < N_out_neighbors; i++){
            if (!received(i)){
                if (received_vout(i)){ //has received a new message
                    received(i) = true;
                    received_vout(i) = false; 
                    int idx = 0;
                    for (int j = 0; j < v_out_msg_recv_buff[i].val_length; j++){
                        idx = v_out_msg_recv_buff[i].idx[j]; //  v_out_msg_idx_first_received[i][j];
                        auto og_idx = idx_to_og_idx.find(idx);
                        if (og_idx != idx_to_og_idx.end()){
                            XV[og_idx->second].push_back(v_out_msg_recv_buff[i].value[j]); //careful: the order in XV will depend on the order in which the messages arrive. But that does not affect the average value.
                        }
                    }                    
                }
                // else{
                    // std::cout << "I am agent " << my_id << " and I am waiting for a message on " << v_out_subscriber[i]->get_topic_name() << std::endl; 
                // } 
            }
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        // for (int i = 0; i < N_in_neighbors; i++){
        //     std::cout << "I am agent " << my_id << "and I am publishing to " << v_in_publisher[i]->get_topic_name() << std::endl; 
        //     v_in_publisher[i]->publish(v_in_msg[i]); //ros
        // }
    }
    //receive_vout_timer.toc();


    return;
}

void HolohoverDmpcAdmmNode::send_vout_receive_vin(){
    //send originals in vout and receive copies in vin
    //this is done after the averaging step

    //send vout
    for (int i = 0; i < N_out_neighbors; i++){
        v_out_msg[i].seq_number += 1;
        v_out_msg[i].header.frame_id = "body"; 
        v_out_msg[i].header.stamp = this->now();

        int idx_row = 0;
        for (int j = 0; j < nx; j++){
            if (isOriginal[j]){
                v_out_msg[i].value[idx_row] = zbar[j];
                idx_row += 1;
            }
            if (idx_row == v_out_msg[i].val_length){
                break;        
            }
        }
        v_out_publisher[i]->publish(v_out_msg[i]);
    }

    //receive vin
    //attention: save received value directly to zbar (not to v_in!)
    Eigen::Array<bool, Dynamic, 1> received(N_in_neighbors,1);
    received.fill(false);
    int cpy_idx = 0;
    int og_idx = 0;
    while (!received.all()){
        for (int i = 0; i < N_in_neighbors; i++){
            if (!received(i)){
                if (received_vin(i)){
                    received_vin(i) = false;
                    received(i) = true;
                    //move to callback?
                    for (int j = 0; j < v_in_msg_recv_buff[i].val_length; j++){
                        og_idx = v_in_msg_recv_buff[i].idx[j]; //  v_in_msg_idx_first_received[i][j];
                        auto tmp = v_in[i].og_idx_to_cpy_idx.find(og_idx);
                        if (tmp != v_in[i].og_idx_to_cpy_idx.end()){
                            cpy_idx = tmp->second;
                            zbar[cpy_idx] = v_in_msg_recv_buff[i].value[j];
                        }  
                    }
                }
            }
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        // for (int i = 0; i < N_out_neighbors; i++){
        //     std::cout << "I am agent " << my_id << "and I am publishing to " << v_out_publisher[i]->get_topic_name() << std::endl; 
        //     v_out_publisher[i]->publish(v_out_msg[i]); //ros
        //     std::cout << "I am agent " << my_id << "and I am publishing to " << v_in_publisher[i]->get_topic_name() << std::endl; 
        //     v_in_publisher[i]->publish(v_in_msg[i]); //ros
        // }        
    }
    return;
}


void HolohoverDmpcAdmmNode::update_v_in(){
    //update all v_in values with values from z
    for (int i = 0; i < N_in_neighbors; i++){
        for (int j = 0; j < v_in[i].nv; j++){
            v_in[i].val[j] = z[v_in[i].cpy_idx[j]];
        }
    }

    return;
}

void HolohoverDmpcAdmmNode::update_v_out(){
    //update all v_out values with values from zbar
    for (int i = 0; i < N_out_neighbors; i++){
        for (int j = 0; j < v_out[i].nv; j++){
            v_out[i].val[j] = zbar[v_out[i].og_idx[j]];
        }
    }
}


int HolohoverDmpcAdmmNode::update_g_beq(){
    g = sprob.g[my_id];    
    lbA.block(0,0,ng,1) = sprob.beq[my_id];
    ubA.block(0,0,ng,1) = sprob.beq[my_id];
    return 0;
}

int main(int argc, char **argv) {

    // int my_id = atoi(argv[argc]);

    // for (int i = 0; i < argc; i++){
    //     std::cout << "input argument =  " << argv[i]  << std::endl;
    // } 

    // int argc_new = argc - 1;
    // char **argv_new = new char*[argc_new+1]; 
    // for (int i = 0; i < argc_new; i++){
    //     argv_new[i] = argv[i];  
    // } 
    // argv_new[argc_new] = nullptr; 

    // // int my_id = 0;

    rclcpp::init(argc, argv);

    HolohoverDmpcAdmmNode::SharedPtr dmpc_admm_node = std::make_shared<HolohoverDmpcAdmmNode>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(dmpc_admm_node);
    executor.spin();

    // rclcpp::spin(std::make_shared<HolohoverDmpcAdmmNode>());
    rclcpp::shutdown();
    // delete[] argv_new ;
    return 0;
}