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

HolohoverDmpcAdmmNode::HolohoverDmpcAdmmNode() :
        Node("dmpc_node"),
        holohover_props(load_holohover_pros(declare_parameter<std::string>("holohover_props_file"))),
        control_settings(load_control_dmpc_settings(*this)),
        holohover(holohover_props, control_settings.dmpc_period),
        sprob(control_settings.Nagents)
{
    my_id = control_settings.my_id;
    Nagents = control_settings.Nagents;    

    // init state
    state.setZero();
    state_at_ocp_solve.setZero();
    u_acc_curr.setZero();
    u_acc_next.setZero();
    motor_velocities.setZero();
    last_control_signal.setZero();
    u_signal.setZero();


    p = Eigen::VectorXd::Zero(control_settings.nx+control_settings.nu+control_settings.nxd);

    // Dummy QP parameters for checking that ADMM works
    //initial positions
    Vector2d x10; x10 << 0.8, 0.0;
    Vector2d x20; x20 << -0.3, 0.0;
    Vector2d x30; x30 << -0.3, 0.0;
    Vector2d x40; x40 << -0.3, 0.0;

    //desired positions
    Vector2d x1d; x1d << 0.8, 0.0;
    Vector2d x2d; x2d << -0.3, 0.0;
    Vector2d x3d; x3d << -0.3, 0.0;
    Vector2d x4d; x4d << -0.3, 0.0;

    if (my_id == 0){
        p[0] = x10[0]; p[1] = x10[1];
        p[9] = x1d[0]; p[10] = x1d[1];
        p[15] = x2d[0]; p[16] = x2d[1]; 
    } else if (my_id == 1){
        p[0] = x20[0]; p[1] = x20[1];
        p[9] = x2d[0]; p[10] = x2d[1];
        p[15] = x3d[0]; p[16] = x3d[1];     
    } else if (my_id == 2){
        p[0] = x30[0]; p[1] = x30[1];
        p[9] = x3d[0]; p[10] = x3d[1];
        p[15] = x4d[0]; p[16] = x4d[1]; 
    } else if (my_id == 3){
        p[0] = x40[0]; p[1] = x40[1];
        p[9] = x4d[0]; p[10] = x4d[1];
    }
    state(0) = p(0); state(1) = p(1); state(2) = p(2); state(3) = p(3); state(4) = p(4); state(5) = p(5);
    state_ref = p.segment(control_settings.nx+control_settings.nu,control_settings.nxd); //todo
    state_ref_at_ocp_solve = state_ref;
    build_qp();

    nz = sprob.H[my_id].rows();
    ng = sprob.Aeq[my_id].rows();
    nh = sprob.Aineq[my_id].rows();

    RCLCPP_INFO(get_logger(), "nz = %d, ng = %d, nh = %d", nz, ng, nh);

    Ncons = sprob.A[my_id].rows();

    rho = control_settings.rho;
    H_bar = sprob.H[my_id] + rho * MatrixXd::Identity(nz,nz);
    g = sprob.g[my_id];
    ub = sprob.ub[my_id];
    lb = sprob.lb[my_id];

    z       = VectorXd::Zero(nz);
    zbar   = VectorXd::Zero(nz);
    gam     = VectorXd::Zero(nz);

    g_bar = g + gam - rho*zbar;

    //PIQP    
    loc_prob.settings().verbose = false;
    loc_prob.settings().compute_timings = false;
    loc_prob.setup(H_bar.sparseView(), g_bar, sprob.Aeq[my_id].sparseView() , sprob.beq[my_id] , sprob.Aineq[my_id].sparseView() , sprob.bineq[my_id], lb, ub); 
    loc_prob.solve();
    
    
    isOriginal.resize(nz);
    isOriginal.setConstant(false);
    isCopy.resize(nz);
    isCopy.setConstant(false);

    out_neighbors = std::vector<int>(0);
    in_neighbors = std::vector<int>(0);
    numCopies = VectorXi::Zero(nz);    
    

    init_coupling();

    for (int i = 0; i < N_og; i++){
        XV.push_back(std::vector<double>());
        auto idx = og_idx_to_idx.find(i);
        XV[i].resize(numCopies(idx->second)+1); //XV stores local original variable and copies from out-neighbors
    }

    //initialize comms
    // v_out
    receive_vout_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    receive_vout_options.callback_group = receive_vout_cb_group;
    v_out_msg.resize(N_out_neighbors);
    v_out_subscriber.resize(N_out_neighbors);
    v_out_publisher.resize(N_out_neighbors);
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

        std::string v_out_pub_topic = "/dmpc/agent";
        v_out_pub_topic.append(std::to_string(my_id));
        v_out_pub_topic.append("ogforagent");
        v_out_pub_topic.append(std::to_string(neighbor_id));
        v_out_publisher[i] = this->create_publisher<holohover_msgs::msg::HolohoverADMMStamped>(
            v_out_pub_topic,
            rclcpp::SystemDefaultsQoS());
    
        std::string v_out_sub_topic = "/dmpc/agent";
        v_out_sub_topic.append(std::to_string(neighbor_id));
        v_out_sub_topic.append("copyofagent");
        v_out_sub_topic.append(std::to_string(my_id));
        bound_received_vout_callback[i] = std::bind(&HolohoverDmpcAdmmNode::received_vout_callback, this, std::placeholders::_1, i); 
        v_out_subscriber[i] = this->create_subscription<holohover_msgs::msg::HolohoverADMMStamped>(
                            v_out_sub_topic, rclcpp::SystemDefaultsQoS(),
                            bound_received_vout_callback[i],receive_vout_options);
    }

    // v_in
    receive_vin_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    receive_vin_options.callback_group = receive_vin_cb_group;
    v_in_msg.resize(N_in_neighbors);
    v_in_subscriber.resize(N_in_neighbors);
    v_in_publisher.resize(N_in_neighbors);
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

        std::string v_in_pub_topic = "/dmpc/agent";
        v_in_pub_topic.append(std::to_string(my_id));
        v_in_pub_topic.append("copyofagent");
        v_in_pub_topic.append(std::to_string(neighbor_id));
        v_in_publisher[i] = this->create_publisher<holohover_msgs::msg::HolohoverADMMStamped>(
            v_in_pub_topic,
            rclcpp::SystemDefaultsQoS());

        std::string v_in_sub_topic = "/dmpc/agent";
        v_in_sub_topic.append(std::to_string(neighbor_id));
        v_in_sub_topic.append("ogforagent");
        v_in_sub_topic.append(std::to_string(my_id));
        bound_received_vin_callback[i] = std::bind(&HolohoverDmpcAdmmNode::received_vin_callback, this, std::placeholders::_1, i);
        v_in_subscriber[i] = this->create_subscription<holohover_msgs::msg::HolohoverADMMStamped>(
                            v_in_sub_topic, rclcpp::SystemDefaultsQoS(),
                            bound_received_vin_callback[i],receive_vin_options);
    }
       
    v_in_msg_idx_first_received.resize(N_in_neighbors);
    v_out_msg_idx_first_received.resize(N_out_neighbors);

    init_topics();

    dmpc_is_initialized = false;

    log_buffer_size = 1;
    admm_timer.reserve(log_buffer_size);
    reserve_time_measurements(log_buffer_size*control_settings.maxiter);

    x_log = -MatrixXd::Ones(log_buffer_size,control_settings.nx);
    u_log = -MatrixXd::Ones(log_buffer_size,control_settings.nu);
    u_before_conversion_log = -MatrixXd::Ones(log_buffer_size,control_settings.nu);
    xd_log = -MatrixXd::Ones(log_buffer_size,control_settings.nxd);
    mpc_step = 0;
    logged_mpc_steps = 0;
    
    std::time_t t = std::time(0);   // get time now
    std::tm* now = std::localtime(&t);
    
    file_name << ament_index_cpp::get_package_prefix("holohover_dmpc") << "/../../log/dmpc_time_measurement_agent" << my_id << "_" << (now->tm_year + 1900) << '_' << (now->tm_mon + 1) << '_' <<  now->tm_mday << "_" << now->tm_hour << "_" << now->tm_min << "_" << now->tm_sec <<".csv";
    log_file = std::ofstream(file_name.str());
    if (log_file.is_open())
    {
        log_file << "mpc_step, x0_1_, x0_2_, x0_3_, x0_4_, x0_5_, x0_6_, u0_1_, u0_2_, u0_3_, u0bc_1_, u0bc_2_, u0bc_3_, xd_1_, xd_2_, xd_3_, xd_4_, xd_5_, xd_6_, admm_time_us_, admm_iter, admm_iter_time_us_, loc_qp_time_us_, zcomm_time_us_, zbarcomm_time_us_, sendvin_time_us_, receivevout_time_us_\n";
        log_file.close();
    }

}

HolohoverDmpcAdmmNode::~HolohoverDmpcAdmmNode()
{

}

void HolohoverDmpcAdmmNode::init_topics()
{
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControlStamped>(
            "control",
            rclcpp::SensorDataQoS());

    HolohoverTrajectory_publisher = this->create_publisher<holohover_msgs::msg::HolohoverTrajectory>(
            "control/HolohoverTrajectory",
            rclcpp::SensorDataQoS());

    state_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_options.callback_group = state_cb_group;        

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverStateStamped>(
            "state", 10,
            std::bind(&HolohoverDmpcAdmmNode::state_callback, this, std::placeholders::_1),state_options);

    state_ref_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_ref_options.callback_group = state_ref_cb_group;
    
    reference_subscription = this->create_subscription<holohover_msgs::msg::HolohoverDmpcStateRefStamped>(
            "dmpc_state_ref", 10,
            std::bind(&HolohoverDmpcAdmmNode::ref_callback, this, std::placeholders::_1),state_ref_options);

    publish_control_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publish_control_options.callback_group = publish_control_cb_group;        

    dmpc_trigger_subscription = this->create_subscription<std_msgs::msg::UInt64>(
            "/dmpc/trigger", 10,
            std::bind(&HolohoverDmpcAdmmNode::publish_control, this, std::placeholders::_1),publish_control_options);
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
        v_in_publisher[i]->publish(v_in_msg[i]);
    }

    //receive vout

    Eigen::Array<bool,Dynamic,1> received(N_out_neighbors,1);
    received.fill(false);
    while (!received.all()){
        for (int i = 0; i < N_out_neighbors; i++){
            if (!received(i)){
                if (received_vout(i)){ //has received a new message
                    received(i) = true;
                    received_vout(i) = false;
                    v_out_msg_idx_first_received[i] = v_out_msg_recv_buff[i].idx;
                }
            }
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        // for (int i = 0; i < N_in_neighbors; i++){
        //     v_in_publisher[i]->publish(v_in_msg[i]); //ros
        // }
        // for (int i = 0; i < N_out_neighbors; i++){
        //     v_out_publisher[i]->publish(v_out_msg[i]); //ros
        // }
    }

    //send vout
    for (int i = 0; i < N_out_neighbors; i++){
        v_out_msg[i].seq_number += 1;
        v_out_msg[i].header.frame_id = "body"; 
        v_out_msg[i].header.stamp = this->now();

        int idx_row = 0;
        for (int j = 0; j < nz; j++){
            if (isOriginal[j]){
                v_out_msg[i].value[idx_row] = zbar[j];
                v_out_msg[i].idx[idx_row] = j;
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
                if (received_vin(i)){
                    received_vin(i) = false;
                    received(i) = true;
                    v_in_msg_idx_first_received[i] = v_in_msg_recv_buff[i].idx;
                }
            }
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        // for (int i = 0; i < N_in_neighbors; i++){
        //     v_in_publisher[i]->publish(v_in_msg[i]); //ros
        // }
        // for (int i = 0; i < N_out_neighbors; i++){
        //     v_out_publisher[i]->publish(v_out_msg[i]); //ros
        // }
    }


    // for (int i = 0; i < N_out_neighbors; i++){
    //     v_out_msg[i].idx_length = 0;
    //     v_out_msg[i].idx.resize(0);  //reduce ADMM message size for solve      
    // }
    // for (int i = 0; i < N_in_neighbors; i++){
    //     v_in_msg[i].idx_length = 0;
    //     v_in_msg[i].idx.resize(0);  //reduce ADMM message size for solve
    // }   

    return;
}

void HolohoverDmpcAdmmNode::publish_control(const std_msgs::msg::UInt64 &publish_control_msg)
{
    std::ignore = publish_control_msg;

    if(!dmpc_is_initialized){
        update_setpoint_in_ocp();
        init_dmpc();
        dmpc_is_initialized = true;
        return;
    }

    // std::cout << "u_acc_curr before conversion = " << u_acc_curr[0] << " , " << u_acc_curr[1] << " , " << u_acc_curr[2] << std::endl;
    u_before_conversion_log.block(mpc_step,0,1,control_settings.nu) = u_acc_curr.transpose();
    // const std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now(); 
    convert_u_acc_to_u_signal();
    u_log.block(mpc_step,0,1,control_settings.nu) = u_acc_curr.transpose();

    // std::cout << "u_acc_curr after conversion = " << u_acc_curr[0] << " , " << u_acc_curr[1] << " , " << u_acc_curr[2] << std::endl; 


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
    // const std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
    // const long duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
    // std::cout << "Signal conversion and publishing duration_us  =" <<duration_us << std::endl;


    update_setpoint_in_ocp();

    
    // const std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
    admm_timer.tic();
    solve(control_settings.maxiter);      
    admm_timer.toc();
    // const std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
    // const long duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
    // std::cout << "ADMM duration_ms  =" <<duration_us/1000 << std::endl;

    get_u_acc_from_sol();

    publish_trajectory();

    x_log.block(mpc_step,0,1,control_settings.nx) = state_at_ocp_solve.transpose(); 
    xd_log.block(mpc_step,0,1,control_settings.nxd) = state_ref_at_ocp_solve.transpose();

    if (mpc_step == log_buffer_size - 1) {
        // const std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
        print_time_measurements();
        clear_time_measurements();
        mpc_step = -1; //gets increased to 0 below
        // const std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
        // const long duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        // std::cout << "Logging duration_us  =" <<duration_us << std::endl;
    } 

    u_acc_curr = u_acc_next;
    mpc_step = mpc_step + 1;
}

void HolohoverDmpcAdmmNode::convert_u_acc_to_u_signal()
{
    motor_velocities = holohover.Ad_motor * motor_velocities + holohover.Bd_motor * last_control_signal;

    // calculate thrust bounds for next step
    Holohover::control_force_t<double> u_force_curr_min, u_force_curr_max;
    holohover.signal_to_thrust<double>(holohover.Ad_motor * motor_velocities + Holohover::control_force_t<double>::Constant(holohover.Bd_motor * holohover_props.idle_signal), u_force_curr_min);
    holohover.signal_to_thrust<double>(holohover.Ad_motor * motor_velocities + Holohover::control_force_t<double>::Constant(holohover.Bd_motor * 1.0), u_force_curr_max);

    // calculate next thrust and motor velocities
    Holohover::control_force_t<double> u_force_curr;
    // Holohover::state_t<double> state_next = holohover.Ad * state + holohover.Bd * u_acc_curr;
    holohover.control_acceleration_to_force(state, u_acc_curr, u_force_curr, u_force_curr_min, u_force_curr_max);
    Holohover::control_force_t<double> motor_velocities_curr;
    holohover.thrust_to_signal(u_force_curr, motor_velocities_curr);

    // calculate control from future motor velocities
    u_signal = (motor_velocities_curr - holohover.Ad_motor * motor_velocities) / holohover.Bd_motor;

    // clip between 0 and 1
    u_signal = u_signal.cwiseMax(holohover_props.idle_signal).cwiseMin(1);    
    holohover.signal_to_thrust(u_signal, u_force_curr);
    holohover.control_force_to_acceleration(state, u_force_curr, u_acc_curr);
    
    // save control inputs for next iterations
    last_control_signal = u_signal;
}

void HolohoverDmpcAdmmNode::publish_trajectory( )
{

    holohover_msgs::msg::HolohoverTrajectory msg;
    msg.header.frame_id = "body";
    msg.header.stamp = this->now();

    msg.state_trajectory.resize(control_settings.N);
    int idx = control_settings.idx_x0;
    for (unsigned int i = 0; i < control_settings.N; i++) //GS: we could also send up to control_settings.N+1
    {
        msg.state_trajectory[i].x = zbar(idx); 
        msg.state_trajectory[i].y = zbar(idx+1);
        msg.state_trajectory[i].v_x = zbar(idx+2);
        msg.state_trajectory[i].v_y = zbar(idx+3);
        msg.state_trajectory[i].yaw = zbar(idx+4);
        msg.state_trajectory[i].w_z = zbar(idx+5);
        idx = idx + 6;
    }

    HolohoverTrajectory_publisher->publish(msg);
}

void HolohoverDmpcAdmmNode::state_callback(const holohover_msgs::msg::HolohoverStateStamped &msg_state)
{
    std::unique_lock state_lock{state_mutex, std::defer_lock};
    state_lock.lock(); 
    state(0) = msg_state.state_msg.x;
    state(1) = msg_state.state_msg.y;
    state(2) = msg_state.state_msg.v_x;
    state(3) = msg_state.state_msg.v_y;
    state(4) = msg_state.state_msg.yaw;
    state(5) = msg_state.state_msg.w_z;
    state_lock.unlock();
}

void HolohoverDmpcAdmmNode::ref_callback(const holohover_msgs::msg::HolohoverDmpcStateRefStamped &ref)
{
    if(ref.val_length != control_settings.nxd){
        RCLCPP_INFO(get_logger(), "Discarded state reference message with incorrect length. Received %d, should be %d", ref.val_length, control_settings.nxd);

        return;
    }
    std::unique_lock state_ref_lock{state_ref_mutex, std::defer_lock};
    state_ref_lock.lock(); 
    for (unsigned int i = 0; i < ref.val_length; i++){
        state_ref(i) = ref.ref_value[i]; 
    }
    state_ref_lock.unlock(); 
}

void HolohoverDmpcAdmmNode::get_u_acc_from_sol()
{
    //u_acc_curr = sol.block(control_settings.idx_u0,0,control_settings.nu,1); //updated in convert_u_acc_to_u_signal
    u_acc_next = zbar.block(control_settings.idx_u1,0,control_settings.nu,1);
}

void HolohoverDmpcAdmmNode::update_setpoint_in_ocp(){

    std::unique_lock state_lock{state_mutex, std::defer_lock};
    state_lock.lock();
    state_at_ocp_solve = state;
    p.segment(0,control_settings.nx) = state_at_ocp_solve;
    state_lock.unlock();                           
    p.segment(control_settings.nx,control_settings.nu) = u_acc_curr;
    std::unique_lock state_ref_lock{state_ref_mutex, std::defer_lock};
    state_ref_lock.lock();
    state_ref_at_ocp_solve = state_ref;
    p.segment(control_settings.nx+control_settings.nu,control_settings.nxd) = state_ref_at_ocp_solve;
    state_ref_lock.unlock();

    std::string function_library = control_settings.folder_name_sprob + "/locFuns.so";

    casadi::DM z_cas;
    casadi::DM p_cas;

    // Use CasADi's "external" to load the compiled function
    casadi::Function f; // = casadi::external("gradFun1","sProb_chain/locFuns.so");
    std::string str;

    std::vector<casadi::DM> arg; // = {z};
    std::vector<casadi::DM> res; // = f(arg);

    int nz_ = g.size();


    VectorXd z = Eigen::VectorXd::Zero(nz_);
  
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

    g = sprob.g[my_id];
   
}

void HolohoverDmpcAdmmNode::init_dmpc()
{
    std::this_thread::sleep_for(std::chrono::seconds(10)); //wait until all subscribers are setup
    init_comms();
    RCLCPP_INFO(get_logger(), "Comms initialized");
    z     = VectorXd::Zero(nz);
    zbar  = VectorXd::Zero(nz);
    gam   = VectorXd::Zero(nz);

    solve(10); //initializes data structures in admm and warm start for first MPC step
    clear_time_measurements();

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

    int nz_ = sprob.A[my_id].cols();


    VectorXd z_ = Eigen::VectorXd::Zero(nz_);
  
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
    sprob.g[my_id] = Eigen::VectorXd::Zero(nz_);
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

    for (int k = 0; k < sprob.lb[my_id].size(); k++){
        if (sprob.lb[my_id][k] == -casadi::inf){
            sprob.lb[my_id][k] = -std::numeric_limits<double>::infinity(); //PIQP
        } else if (sprob.lb[my_id][k] == casadi::inf){
            sprob.lb[my_id][k] = std::numeric_limits<double>::infinity(); //PIQP
        }
    }

    for (int k = 0; k < sprob.ub[my_id].size(); k++){
        if (sprob.ub[my_id][k] == -casadi::inf){
            sprob.ub[my_id][k] = -std::numeric_limits<double>::infinity(); //PIQP
        } else if (sprob.ub[my_id][k] == casadi::inf){
            sprob.ub[my_id][k] = std::numeric_limits<double>::infinity(); //PIQP
        }
    }


}

void HolohoverDmpcAdmmNode::init_coupling()
{
    //Coupling information
    Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Coup = -MatrixXi::Ones(nz,2); //col1: owning agent ID; col2: index in nz of owning agent

    std::vector<int> nnz (Nagents);
    for (int i = 0; i < Nagents; i++){
        nnz[i] = sprob.A[i].cols();
    }
    MatrixXd tmp;
    std::vector<int>::iterator it;
    VectorXd nv_out = VectorXd::Zero(Nagents);

    //Identify coupling structure
    //Out-neighbors
    N_og = 0;
    for (int i = 0; i < nz; i++){
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
                    tmp = MatrixXd::Zero(1,nnz[k]);
                    tmp = sprob.A[k].block(j,0,1,nnz[k]);
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
    for (int i = 0; i < nz; i++){
        isSource = 0;
        for (int j = 0; j < Ncons; j++){
            if(sprob.A[my_id](j,i) <= -0.9){
                isCopy(i) = true;
                for (int k = 0; k < Nagents; k++){
                    tmp.resize(1,nnz[k]);
                    tmp = sprob.A[k].block(j,0,1,nnz[k]);
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
    isCopy.resize(nz);

    for (int i = 0; i < nz; i++){
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

    //In-neighbors
    int nij = 0;
    for (int j = 0; j < N_in_neighbors; j++){
        v_in.push_back(vCpy());
        nij = 0;
        for (int i = 0; i < nz; i++){
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
        for (int i = 0; i < nz; i++){
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
        for (int i = 0; i < nz; i++){
            bool isDestination = 0;
            for (int c = 0; c < Ncons; c++){
                if (sprob.A[my_id](c,i) == 1){
                    tmp.resize(1,nnz[k]);
                    tmp = sprob.A[k].block(c,0,1,nnz[k]);
                    isDestination = (tmp.array() == -1.0).any();
                    if (isDestination){
                        for (int l = 0; l < nnz[k]; l++){
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
    
    v_in_msg_recv_buff.resize(N_in_neighbors); 
    v_out_msg_recv_buff.resize(N_out_neighbors);

    return;
}

int HolohoverDmpcAdmmNode::solve(unsigned int maxiter_)
{  
    for (unsigned int iter = 0; iter < maxiter_; iter++){

        // std::cout << "Starting ADMM iteration " << iter << std::endl;
        iter_timer.tic(),

        //Step 1: local z update
        loc_timer.tic();
        g_bar = g + gam - rho*zbar;

        //PIQP
        loc_prob.update(piqp::nullopt, g_bar, piqp::nullopt , sprob.beq[my_id] , piqp::nullopt , piqp::nullopt, piqp::nullopt, piqp::nullopt,true);
        loc_prob.solve();
        z = loc_prob.result().x;

        for (int i = 0; i < N_og; i++){
            auto idx = og_idx_to_idx.find(i);
            XV[i].clear();
            XV[i].push_back(z(idx->second));
        }
        loc_timer.toc();

        //Communication
        z_comm_timer.tic();
        update_v_in();
        send_vin_receive_vout();
        z_comm_timer.toc();

        //Step 2: averaging
        double avg;
        for (int i = 0; i < N_og; i++){
            avg = std::accumulate(XV[i].begin(), XV[i].end(), 0.0) / XV[i].size();
            auto idx = og_idx_to_idx.find(i);
            zbar[idx->second] = avg;
        }

        //Communication
        zbar_comm_timer.tic();
        update_v_out();
        send_vout_receive_vin();
        zbar_comm_timer.toc();

        // Step 3: dual update
        gam = gam + rho*(z-zbar);  
        iter_timer.toc();

    }
    
    return 0;
}

void HolohoverDmpcAdmmNode::received_vin_callback(const holohover_msgs::msg::HolohoverADMMStamped &v_in_msg_, int in_neighbor_idx_){
    if(!received_vin(in_neighbor_idx_)){     
        v_in_msg_recv_buff[in_neighbor_idx_] = v_in_msg_;
        received_vin(in_neighbor_idx_) = true;
    }
}

void HolohoverDmpcAdmmNode::received_vout_callback(const holohover_msgs::msg::HolohoverADMMStamped &v_out_msg_, int out_neighbor_idx_){
    if(!received_vout(out_neighbor_idx_)){    
        v_out_msg_recv_buff[out_neighbor_idx_] = v_out_msg_;
        received_vout(out_neighbor_idx_) = true; 
    }  
} 

void HolohoverDmpcAdmmNode::send_vin_receive_vout(){
    //send copies in vin and receive copies in vout
    //this is done before the averaging step

    //send vin
    send_vin_timer.tic();
    for (int i = 0; i < N_in_neighbors; i++){
        v_in_msg[i].seq_number += 1;
        v_in_msg[i].header.frame_id = "body"; 
        v_in_msg[i].header.stamp = this->now(); 
        Eigen::VectorXd::Map(&v_in_msg[i].value[0], v_in[i].val.size()) = v_in[i].val;
        v_in_publisher[i]->publish(v_in_msg[i]); //ros
    }
    send_vin_timer.toc();
    //receive vout
    receive_vout_timer.tic();
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
                        idx = v_out_msg_idx_first_received[i][j];
                        auto og_idx = idx_to_og_idx.find(idx);
                        if (og_idx != idx_to_og_idx.end()){
                            XV[og_idx->second].push_back(v_out_msg_recv_buff[i].value[j]); //careful: the order in XV will depend on the order in which the messages arrive. But that does not affect the average value.
                        }
                    }                    
                }
            }
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        // for (int i = 0; i < N_in_neighbors; i++){
        //     v_in_publisher[i]->publish(v_in_msg[i]); //ros
        // }
    }
    receive_vout_timer.toc();


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
        for (int j = 0; j < nz; j++){
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
                    for (int j = 0; j < v_in_msg_recv_buff[i].val_length; j++){
                        og_idx = v_in_msg_idx_first_received[i][j];
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
        //     v_out_publisher[i]->publish(v_out_msg[i]); //ros
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

void HolohoverDmpcAdmmNode::print_time_measurements(){
    
      

    unsigned int N_rows = iter_timer.m_log.size();
    int k = 0; //MPC step
    unsigned int row = 0;
    while (row < N_rows){
        for (unsigned int i = 0; i < control_settings.maxiter; i++){     
            log_file.open(file_name.str(),std::ios_base::app);
            if (log_file.is_open())
            {
                log_file << logged_mpc_steps+k << "," << x_log(k,0) << "," << x_log(k,1) << "," << x_log(k,2) << "," << x_log(k,3) << "," << x_log(k,4) << "," << x_log(k,5) << "," << u_log(k,0) << "," << u_before_conversion_log(k,1) << "," << u_before_conversion_log(k,2) << "," << u_before_conversion_log(k,0) << "," << u_log(k,1) << "," << u_log(k,2) << "," << xd_log(k,0) << "," << xd_log(k,1) << "," << xd_log(k,2) << "," << xd_log(k,3) << "," << xd_log(k,4) << "," << xd_log(k,5) << "," << admm_timer.m_log[k] << "," << i << "," << iter_timer.m_log[row] << "," << loc_timer.m_log[row] << "," << z_comm_timer.m_log[row] << "," << zbar_comm_timer.m_log[row] << "," << send_vin_timer.m_log[row] << "," << receive_vout_timer.m_log[row] << "\n";        
            }
            log_file.close();

            row++;
        }
        k++;
    }
    logged_mpc_steps = logged_mpc_steps + k;

    
    return;
}

void HolohoverDmpcAdmmNode::reserve_time_measurements(unsigned int new_cap){
    loc_timer.reserve(new_cap);
    iter_timer.reserve(new_cap);
    z_comm_timer.reserve(new_cap);
    zbar_comm_timer.reserve(new_cap);
    send_vin_timer.reserve(new_cap);
    receive_vout_timer.reserve(new_cap);
}

void HolohoverDmpcAdmmNode::clear_time_measurements(){
    admm_timer.clear();
    iter_timer.clear();
    loc_timer.clear();
    z_comm_timer.clear();
    zbar_comm_timer.clear();
    send_vin_timer.clear();
    receive_vout_timer.clear();
    return;
}


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    HolohoverDmpcAdmmNode::SharedPtr dmpc_admm_node = std::make_shared<HolohoverDmpcAdmmNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(dmpc_admm_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}