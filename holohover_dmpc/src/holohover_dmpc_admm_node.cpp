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

using namespace Eigen;

HolohoverDmpcAdmmNode::HolohoverDmpcAdmmNode() :
        Node("dmpc_node"),
        holohover_props(load_holohover_pros(declare_parameter<std::string>("holohover_props_file"))),
        control_settings(load_control_dmpc_settings(*this)),
        holohover(holohover_props, control_settings.control_period > 0
                                   ? control_settings.control_period
                                   : control_settings.dmpc_period),
        sprob(control_settings.Nagents)
{


    RCLCPP_INFO(get_logger(), "DMPC NODE: list of obstacles: ");
    for (const auto &obstacle : control_settings.obstacles)
        RCLCPP_INFO(get_logger(), "\t\tobstacle: %s", obstacle.c_str());

    my_id = control_settings.my_id;
    Nagents = control_settings.Nagents;    

    // init state
    state.setZero();
    state_at_ocp_solve.setZero();
    u_acc_curr.setZero();
    u_acc_next.setZero();
    u_signal.setZero();

    motor_velocities.setZero();
    last_control_signal.setZero();

    dist = Eigen::Vector3d::Zero();
    dist_at_ocp_solve = dist;


    p = Eigen::VectorXd::Zero(control_settings.nx+control_settings.nu+3+control_settings.nxd+control_settings.nud);

    // Dummy QP parameters for checking that ADMM works
    //initial positions
    Vector2d x10; x10 << 0.5, 0.0;
    Vector2d x20; x20 << -0.3, 0.0;
    Vector2d x30; x30 << -0.3, 0.0;
    Vector2d x40; x40 << -0.3, 0.0;

    //desired positions
    Vector2d x1d; x1d << 0.5, 0.0;
    Vector2d x2d; x2d << -0.3, 0.0;
    Vector2d x3d; x3d << -0.3, 0.0;
    Vector2d x4d; x4d << -0.3, 0.0;

    if (my_id == 0){
        p[0] = x10[0]; p[1] = x10[1];
        p[9+3] = x1d[0]; p[10+3] = x1d[1];
        p[15+3] = x2d[0]; p[16+3] = x2d[1]; 
    } else if (my_id == 1){
        p[0] = x20[0]; p[1] = x20[1];
        p[9+3] = x2d[0]; p[10+3] = x2d[1];
        p[15+3] = x3d[0]; p[16+3] = x3d[1];     
    } else if (my_id == 2){
        p[0] = x30[0]; p[1] = x30[1];
        p[9+3] = x3d[0]; p[10+3] = x3d[1];
        p[15+3] = x4d[0]; p[16+3] = x4d[1]; 
    } else if (my_id == 3){
        p[0] = x40[0]; p[1] = x40[1];
        p[9+3] = x4d[0]; p[10+3] = x4d[1];
    }
    state(0) = p(0); state(1) = p(1); state(2) = p(2); state(3) = p(3); state(4) = p(4); state(5) = p(5);
    state_at_ocp_solve = state;
    state_ref = p.segment(control_settings.nx+control_settings.nu+3,control_settings.nxd); //todo
    state_ref_at_ocp_solve = state_ref;
    input_ref = Eigen::VectorXd::Zero(control_settings.nud);
    build_qp();

    nz = sprob.H[my_id].res[0].rows();
    ng = sprob.Aeq[my_id].res[0].rows();
    nh = sprob.Aineq[my_id].res[0].rows();

    RCLCPP_INFO(get_logger(), "nz = %d, ng = %d, nh = %d", nz, ng, nh);

    Ncons = sprob.A[my_id].rows();

    rho = control_settings.rho;
    H_bar = sprob.H[my_id].res[0];
    for (int i = 0; i < nz; i++) {
        H_bar.coeffRef(i, i) += rho;
    }
    H_bar.makeCompressed();

    z       = VectorXd::Zero(nz);
    zbar    = VectorXd::Zero(nz);
    gam     = VectorXd::Zero(nz);

    g_bar = sprob.g[my_id].res[0] + gam - rho*zbar;

    //PIQP    
    loc_prob.settings().verbose = false;
    loc_prob.settings().compute_timings = false;
    loc_prob.setup(H_bar, g_bar, sprob.Aeq[my_id].res[0], sprob.beq[my_id].res[0], sprob.Aineq[my_id].res[0], sprob.bineq[my_id].res[0], sprob.lb[my_id], sprob.ub[my_id]);
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
        XV[i].resize(2*(numCopies(idx->second)+1)); //XV stores local original variable and copies from out-neighbors and dual variables
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
        v_out_msg[i].gam_length = 0; //only send dual variables with v_in msg (before averaging)
        v_out_msg[i].gamma.resize(v_out_msg[i].gam_length);  
        v_out_msg[i].idx.resize(v_out_msg[i].idx_length);
        v_out_msg[i].seq_number = 0;
        for (int idx_row = 0; idx_row<v_out_msg[i].val_length; idx_row++) {
            v_out_msg[i].value[idx_row] = std::numeric_limits<double>::quiet_NaN();
        }
        v_out_msg[i].id_sender = my_id;

        std::ostringstream v_out_pub_topic;
        v_out_pub_topic << "dmpc/ogforagent" << neighbor_id;
        v_out_publisher[i] = this->create_publisher<holohover_msgs::msg::HolohoverADMMStamped>(
            v_out_pub_topic.str(),
            rclcpp::SystemDefaultsQoS());
    
        std::ostringstream v_out_sub_topic;
        v_out_sub_topic << "/h" << neighbor_id << "/dmpc/copyofagent" << my_id;
        bound_received_vout_callback[i] = std::bind(&HolohoverDmpcAdmmNode::received_vout_callback, this, std::placeholders::_1, i);

        v_out_subscriber[i] = this->create_subscription<holohover_msgs::msg::HolohoverADMMStamped>(
                            v_out_sub_topic.str(), rclcpp::SystemDefaultsQoS(),
                            bound_received_vout_callback[i],receive_vout_options);
        // rclcpp::QoS(1)
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
        v_in_msg[i].gam_length = v_in[i].nv;  
        v_in_msg[i].gamma.resize(v_in_msg[i].gam_length); 
        v_in_msg[i].idx_length = v_in[i].nv;
        v_in_msg[i].idx.resize(v_in_msg[i].idx_length);
        Eigen::VectorXi::Map(&v_in_msg[i].idx[0], v_in[i].og_idx.size()) = v_in[i].og_idx;
        v_in_msg[i].seq_number = 0;
        for (int idx_row = 0; idx_row<v_in_msg[i].val_length; idx_row++) {
            v_in_msg[i].value[idx_row] = std::numeric_limits<double>::quiet_NaN();
            v_in_msg[i].gamma[idx_row] = std::numeric_limits<double>::quiet_NaN();
        }
        v_in_msg[i].id_sender = my_id;

        std::ostringstream v_in_pub_topic;
        v_in_pub_topic << "dmpc/copyofagent" << neighbor_id;
        v_in_publisher[i] = this->create_publisher<holohover_msgs::msg::HolohoverADMMStamped>(
            v_in_pub_topic.str(),
            rclcpp::SystemDefaultsQoS());

        std::ostringstream v_in_sub_topic;
        v_in_sub_topic << "/h" << neighbor_id << "/dmpc/ogforagent" << my_id;
        bound_received_vin_callback[i] = std::bind(&HolohoverDmpcAdmmNode::received_vin_callback, this, std::placeholders::_1, i);
        v_in_subscriber[i] = this->create_subscription<holohover_msgs::msg::HolohoverADMMStamped>(
                            v_in_sub_topic.str(), rclcpp::SystemDefaultsQoS(),
                            bound_received_vin_callback[i],receive_vin_options);
    }
       
    v_in_msg_idx_first_received.resize(N_in_neighbors);
    v_out_msg_idx_first_received.resize(N_out_neighbors);


    init_topics();

    dmpc_is_initialized = false;

    log_buffer_size = 1;
    admm_timer.reserve(log_buffer_size);
    reserve_time_measurements(log_buffer_size*control_settings.maxiter);
    convert_u_acc_timer.reserve(log_buffer_size);
    publish_signal_timer.reserve(log_buffer_size);
    update_setpoint_timer.reserve(log_buffer_size);
    get_state_timer.reserve(log_buffer_size);

    x_log = -MatrixXd::Ones(log_buffer_size,control_settings.nx);
    u_log = -MatrixXd::Ones(log_buffer_size,control_settings.nu);
    u_before_conversion_log = -MatrixXd::Ones(log_buffer_size,control_settings.nu);
    xd_log = -MatrixXd::Ones(log_buffer_size,control_settings.nx); //not nxd, because we only store the current setpoint if xd is a trajectory
    ud_log = -MatrixXd::Ones(log_buffer_size,control_settings.nu);
    z_async = -MatrixXi::Ones(log_buffer_size,control_settings.maxiter);
    zbar_async = -MatrixXi::Ones(log_buffer_size,control_settings.maxiter);
    mpc_step = 0;
    mpc_step_since_log = 0;
    logged_mpc_steps = 0;
    xd_ref_idx = 0;
    ud_ref_idx = 0;
    
    std::time_t t = std::time(0);   // get time now
    std::tm* now = std::localtime(&t);
    
    std::ostringstream file_name;
    file_name << ament_index_cpp::get_package_prefix("holohover_dmpc") << "/../../log/dmpc_time_measurement_agent" << my_id << "_" << (now->tm_year + 1900) << '_' << (now->tm_mon + 1) << '_' <<  now->tm_mday << "_" << now->tm_hour << "_" << now->tm_min << "_" << now->tm_sec <<".csv";
    quill::Backend::start();

    auto file_sink = quill::Frontend::create_or_get_sink<quill::FileSink>(file_name.str());

    quill_logger = quill::Frontend::create_or_get_logger("root", std::move(file_sink), "%(message)");
    QUILL_LOG_INFO(quill_logger, "mpc_step, x0_1_, x0_2_, x0_3_, x0_4_, x0_5_, x0_6_, u0_1_, u0_2_, u0_3_, u0bc_1_, u0bc_2_, u0bc_3_, xd_1_, xd_2_, xd_3_, xd_4_, xd_5_, xd_6_, ud_1_, ud_2_, ud_3_, get_state_time_us_, convert_uacc_time_us_, publish_signal_time_us_, update_setpoint_time_us_, admm_time_us_, admm_iter, admm_iter_time_us_, loc_qp_time_us_, zcomm_time_us_, zbarcomm_time_us_, sendvin_time_us_, receivevout_time_us_, z_is_async, zbar_is_async");

    std::ostringstream file_name_sol;
    file_name_sol << ament_index_cpp::get_package_prefix("holohover_dmpc") << "/../../log/dmpc_sol_log_agent" << my_id << "_" << (now->tm_year + 1900) << '_' << (now->tm_mon + 1) << '_' <<  now->tm_mday << "_" << now->tm_hour << "_" << now->tm_min << "_" << now->tm_sec <<".csv";
    auto file_sink_sol = quill::Frontend::create_or_get_sink<quill::FileSink>(file_name_sol.str());
    file_sink_sol = quill::Frontend::create_or_get_sink<quill::FileSink>(file_name_sol.str());
    sol_logger = quill::Frontend::create_or_get_logger("sol_logger", std::move(file_sink_sol), "%(message)");
    
    if (!control_settings.file_name_xd_trajectory.empty()){
        sprob.csvRead(xd_ref,control_settings.file_name_xd_trajectory,20);
    }

    if (!control_settings.file_name_ud_trajectory.empty()){
        sprob.csvRead(ud_ref,control_settings.file_name_ud_trajectory,20);
        p.segment(control_settings.nx+control_settings.nu+3+control_settings.nxd,control_settings.nud) = input_ref;
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

    // HolohoverTrajectory_publisher = this->create_publisher<holohover_msgs::msg::HolohoverTrajectory>(
    //         "control/HolohoverTrajectory",
    //         rclcpp::SensorDataQoS());

    state_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_options.callback_group = state_cb_group;        

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverStateDisturbanceStamped>(
            "state_disturbance", 10,
            std::bind(&HolohoverDmpcAdmmNode::state_callback, this, std::placeholders::_1),state_options);

    state_ref_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_ref_options.callback_group = state_ref_cb_group;
    
    reference_subscription = this->create_subscription<holohover_msgs::msg::HolohoverDmpcStateRefStamped>(
            "dmpc_state_ref", 10,
            std::bind(&HolohoverDmpcAdmmNode::ref_callback, this, std::placeholders::_1),state_ref_options);

    mpc_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    dmpc_trigger_subscription = this->create_subscription<std_msgs::msg::UInt64>(
            "/dmpc/trigger", 10,
            std::bind(&HolohoverDmpcAdmmNode::init_dmpc, this, std::placeholders::_1));
}

void HolohoverDmpcAdmmNode::init_comms(){
    //call this before you call solve()
    //send around the indices of v_inMsg and v_outMsg one. Then set idx_length to zero for future messages.

    //send vin
    for (int i = 0; i < N_in_neighbors; i++){
        v_in_msg[i].seq_number += 1;
        v_in_msg[i].header.frame_id = "body"; 
        v_in_msg[i].header.stamp = this->now(); 
        Eigen::VectorXf::Map(&v_in_msg[i].value[0], v_in[i].val.size()) = v_in[i].val.cast<float>();
        Eigen::VectorXf::Map(&v_in_msg[i].gamma[0], v_in[i].gam.size()) = v_in[i].gam.cast<float>();
        v_in_publisher[i]->publish(v_in_msg[i]);
    }

    //receive vout

    Eigen::Array<bool,Dynamic,1> received(N_out_neighbors,1);
    received.fill(false); 
    while (!received.all()){
        std::unique_lock<std::mutex> lock(v_out_mutex);
        for (int i = 0; i < N_out_neighbors; i++){
            if (!received(i)){                
                if (v_out_msg_recv_queue[i].size() > 0) {
                    received(i) = true;
                    v_out_msg_recv_buff[i] = v_out_msg_recv_queue[i][0];
                    v_out_msg_recv_queue[i].clear();
                    v_out_msg_idx_first_received[i] = v_out_msg_recv_buff[i].idx;
                }
            }
        }
        if(!received.all()){
            v_out_cv.wait_for(lock,std::chrono::milliseconds(10));
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
        std::unique_lock<std::mutex> lock(v_in_mutex);
        for (int i = 0; i < N_in_neighbors; i++){
            if (!received(i)){
                if (v_in_msg_recv_queue[i].size() > 0){
                    received(i) = true;
                    v_in_msg_recv_buff[i] = v_in_msg_recv_queue[i][0];
                    v_in_msg_recv_queue[i].clear();
                    v_in_msg_idx_first_received[i] = v_in_msg_recv_buff[i].idx;
                }
            }
        }
        if(!received.all()){
            v_in_cv.wait_for(lock,std::chrono::milliseconds(10));
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        // for (int i = 0; i < N_in_neighbors; i++){
        //     v_in_publisher[i]->publish(v_in_msg[i]); //ros
        // }
        // for (int i = 0; i < N_out_neighbors; i++){
        //     v_out_publisher[i]->publish(v_out_msg[i]); //ros
        // }
    }


    for (int i = 0; i < N_out_neighbors; i++){
        v_out_msg[i].idx_length = 0;
        v_out_msg[i].idx.resize(0);  //reduce ADMM message size for solve      
    }
    for (int i = 0; i < N_in_neighbors; i++){
        v_in_msg[i].idx_length = 0;
        v_in_msg[i].idx.resize(0);  //reduce ADMM message size for solve
    }   

    return;
}

void HolohoverDmpcAdmmNode::run_dmpc()
{
    std::unique_lock u_acc_curr_lock{u_acc_curr_mutex, std::defer_lock};
    u_acc_curr_lock.lock();
    u_acc_curr = u_acc_next;
    u_acc_curr_lock.unlock();

    get_state_timer.tic();
    std::unique_lock state_lock{state_mutex, std::defer_lock};
    state_lock.lock();
    state_at_ocp_solve = state;
    dist_at_ocp_solve = dist;
    state_lock.unlock();
    get_state_timer.toc();
  
    convert_u_acc_timer.tic(); //this timestamp is also used to check whether admm should be terminated
    u_before_conversion_log.block(mpc_step_since_log,0,1,control_settings.nu) = u_acc_curr.transpose();
    convert_u_acc_to_u_signal();
    u_log.block(mpc_step_since_log,0,1,control_settings.nu) = u_acc_curr.transpose();
    convert_u_acc_timer.toc();

    publish_signal_timer.tic();
    if (control_settings.control_period == 0) {
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
    }
    publish_signal_timer.toc();

    update_setpoint_timer.tic();
    update_setpoint_in_ocp();
    update_setpoint_timer.toc();
    
    admm_timer.tic();
    solve(control_settings.maxiter,false);      
    admm_timer.toc();

    get_u_acc_from_sol();


    //publish_trajectory();

    x_log.block(mpc_step_since_log,0,1,control_settings.nx) = state_at_ocp_solve.transpose(); 
    xd_log.block(mpc_step_since_log,0,1,control_settings.nx) = state_ref_at_ocp_solve.transpose().segment(0,control_settings.nx);
    
    if (!control_settings.file_name_ud_trajectory.empty()){
        ud_log.block(mpc_step_since_log,0,1,control_settings.nu) = input_ref.transpose().segment(0,control_settings.nu);
    }

    if (mpc_step_since_log == log_buffer_size - 1) {
        print_time_measurements();
        clear_time_measurements();
        mpc_step_since_log = -1; //gets increased to 0 below
    }

    std::string print_line_;
    for (int i = 0; i < zbar.size(); i++){
        if(i > 0){
            print_line_ += ",";
        } 
        print_line_ += std::to_string(zbar(i));
    }
    QUILL_LOG_INFO(sol_logger, "{},{}",mpc_step,print_line_);

    mpc_step_since_log = mpc_step_since_log + 1;
    mpc_step = mpc_step + 1;
}

void HolohoverDmpcAdmmNode::publish_control()
{
    Holohover::control_acc_t<double> u_acc_set_point;
    std::unique_lock u_acc_curr_lock{u_acc_curr_mutex, std::defer_lock};
    u_acc_curr_lock.lock();
    u_acc_set_point = u_acc_curr;
    u_acc_curr_lock.unlock();

    Holohover::state_t<double> current_state;
    std::unique_lock state_lock{state_mutex, std::defer_lock};
    state_lock.lock();
    current_state = state;
    state_lock.unlock();

    motor_velocities = holohover.Ad_motor * motor_velocities + holohover.Bd_motor * last_control_signal;

    Holohover::control_force_t<double> u_force_at_publish;
    Holohover::control_acc_t<double> u_acc_at_publish;
    holohover.signal_to_thrust(motor_velocities, u_force_at_publish);
    holohover.control_force_to_acceleration(state, u_force_at_publish, u_acc_at_publish);

    // calculate control for next step
    Holohover::state_t<double> state_next = holohover.Ad * state + holohover.Bd * u_acc_at_publish;

    // calculate thrust bounds for next step
    Holohover::control_force_t<double> u_force_next_min, u_force_next_max;
    holohover.signal_to_thrust<double>(holohover.Ad_motor * motor_velocities + Holohover::control_force_t<double>::Constant(holohover.Bd_motor * holohover_props.idle_signal), u_force_next_min);
    holohover.signal_to_thrust<double>(holohover.Ad_motor * motor_velocities + Holohover::control_force_t<double>::Constant(holohover.Bd_motor * 1.0), u_force_next_max);

    // calculate next thrust and motor velocities
    Holohover::control_force_t<double> u_force_next;
    holohover.control_acceleration_to_force(state_next, u_acc_set_point, u_force_next, u_force_next_min, u_force_next_max);
    Holohover::control_force_t<double> motor_velocities_next;
    holohover.thrust_to_signal(u_force_next, motor_velocities_next);

    // calculate control from future motor velocities
    Holohover::control_force_t<double> u_signal_publish = (motor_velocities_next - holohover.Ad_motor * motor_velocities) / holohover.Bd_motor;

    // clip between 0 and 1
    u_signal_publish = u_signal_publish.cwiseMax(holohover_props.idle_signal).cwiseMin(1);

    holohover_msgs::msg::HolohoverControlStamped control_msg;
    control_msg.header.frame_id = "body";
    control_msg.header.stamp = this->now();
    control_msg.motor_a_1 = u_signal_publish(0);
    control_msg.motor_a_2 = u_signal_publish(1);
    control_msg.motor_b_1 = u_signal_publish(2);
    control_msg.motor_b_2 = u_signal_publish(3);
    control_msg.motor_c_1 = u_signal_publish(4);
    control_msg.motor_c_2 = u_signal_publish(5);
    control_publisher->publish(control_msg);

    // save control inputs for next iterations
    last_control_signal = u_signal_publish;
}

void HolohoverDmpcAdmmNode::convert_u_acc_to_u_signal()
{
    //convert to signal
    Holohover::control_force_t<double> u_force_curr;
    holohover.control_acceleration_to_force(state_at_ocp_solve, u_acc_curr, u_force_curr);
    holohover.thrust_to_signal(u_force_curr, u_signal);

    // clip between 0 and 1
    u_signal = u_signal.cwiseMax(holohover_props.idle_signal).cwiseMin(1);    
    holohover.signal_to_thrust(u_signal, u_force_curr);

    //convert back to acceleration for OCP
    holohover.control_force_to_acceleration(state_at_ocp_solve, u_force_curr, u_acc_curr);
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

void HolohoverDmpcAdmmNode::state_callback(const holohover_msgs::msg::HolohoverStateDisturbanceStamped &msg_state)
{
    std::unique_lock state_lock{state_mutex, std::defer_lock};
    state_lock.lock(); 
    state(0) = msg_state.state_msg.x;
    state(1) = msg_state.state_msg.y;
    state(2) = msg_state.state_msg.v_x;
    state(3) = msg_state.state_msg.v_y;
    state(4) = msg_state.state_msg.yaw;
    state(5) = msg_state.state_msg.w_z;
    dist(0) = msg_state.state_msg.dist_x;
    dist(1) = msg_state.state_msg.dist_y;
    dist(2) = msg_state.state_msg.dist_yaw;
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

    p.segment(0,control_settings.nx) = state_at_ocp_solve;
    p.segment(control_settings.nx,control_settings.nu) = u_acc_curr;
    p.segment(control_settings.nx+control_settings.nu,3) = dist_at_ocp_solve;
    std::unique_lock state_ref_lock{state_ref_mutex, std::defer_lock};
    state_ref_lock.lock();
    
    if (!control_settings.file_name_xd_trajectory.empty() && xd_ref_idx < xd_ref.rows()){
        if (mpc_step == std::floor(xd_ref(xd_ref_idx,0))){
            state_ref = (xd_ref.block(xd_ref_idx,1,1,control_settings.nxd)).transpose();
            xd_ref_idx = xd_ref_idx + 1;
        }  
    }

    state_ref_at_ocp_solve = state_ref;
    state_ref_lock.unlock();

    p.segment(control_settings.nx+control_settings.nu+3,control_settings.nxd) = state_ref_at_ocp_solve;

    if (!control_settings.file_name_ud_trajectory.empty() && ud_ref_idx < ud_ref.rows()){
        if (mpc_step == std::floor(ud_ref(ud_ref_idx,0))){
            input_ref = (ud_ref.block(ud_ref_idx,1,1,control_settings.nud)).transpose();
            ud_ref_idx = ud_ref_idx + 1;
        }
        p.segment(control_settings.nx+control_settings.nu+3+control_settings.nxd,control_settings.nud) = input_ref;
    }

    int nz_ = sprob.g[my_id].res[0].size();
    VectorXd z_ = Eigen::VectorXd::Zero(nz_);

    ///g
    sprob.g[my_id].set_arg(0, z_.data(), z_.size());
    sprob.g[my_id].set_arg(1, p.data(), p.size());
    sprob.g[my_id].eval();

    //beq
    sprob.beq[my_id].set_arg(0, z_.data(), z_.size());
    sprob.beq[my_id].set_arg(1, p.data(), p.size());
    sprob.beq[my_id].eval();
    sprob.beq[my_id].res[0] = -sprob.beq[my_id].res[0];
}

void HolohoverDmpcAdmmNode::init_dmpc(const std_msgs::msg::UInt64 &publish_control_msg)
{

    std::ignore = publish_control_msg;
    if(!dmpc_is_initialized){
        std::this_thread::sleep_for(std::chrono::seconds(1)); //wait until all subscribers are setup
        std::unique_lock state_lock{state_mutex, std::defer_lock};
        state_lock.lock();
        state_at_ocp_solve = state;
        state_lock.unlock();
        update_setpoint_in_ocp();
        RCLCPP_INFO(get_logger(), "Initializing comms");
        init_comms();
        RCLCPP_INFO(get_logger(), "Comms initialized");
        z     = VectorXd::Zero(nz);
        zbar  = VectorXd::Zero(nz);
        gam   = VectorXd::Zero(nz);

        solve(10,true); //initializes data structures in admm and warm start for first MPC step
        clear_time_measurements();

        mpc_timer = rclcpp::create_timer(
            this, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME),
            std::chrono::duration<double>(control_settings.dmpc_period),
            std::bind(&HolohoverDmpcAdmmNode::run_dmpc, this), mpc_cb_group);

        if (control_settings.control_period > 0) {
            control_timer = this->create_wall_timer(
                    std::chrono::duration<double>(control_settings.control_period),
                    std::bind(&HolohoverDmpcAdmmNode::publish_control, this), control_cb_group);
        }

        int64_t dt = (int64_t) (control_settings.dmpc_period * 1e9);
        int64_t time_since_epoch = get_last_call_time(mpc_timer->get_timer_handle().get());
        int64_t offset = time_since_epoch / dt;
        int64_t t_wake = (offset + 3) * dt;
        set_next_call_time(const_cast<rcl_timer_t*>(mpc_timer->get_timer_handle().get()), t_wake);
        RCLCPP_INFO(get_logger(), "First timer callback at %ld", t_wake);

        dmpc_is_initialized = true;
    } 

    return;
}

void HolohoverDmpcAdmmNode::build_qp()
{
    //init sprob
    std::string functionLibrary = control_settings.folder_name_sprob + "/locFuns.so";

    // H
    std::string str = "HessFfun" + std::to_string(my_id + 1); //convert to matlab index
    sprob.H[my_id].init(casadi::external(str, functionLibrary));

    // g
    str = "gradFun" + std::to_string(my_id + 1); //convert to matlab index
    sprob.g[my_id].init(casadi::external(str, functionLibrary));

    //Aeq
    str = "JGfun" + std::to_string(my_id + 1); //convert to matlab index
    sprob.Aeq[my_id].init(casadi::external(str, functionLibrary));

    //Aineq
    str = "JHfun" + std::to_string(my_id + 1); //convert to matlab index
    sprob.Aineq[my_id].init(casadi::external(str, functionLibrary));

    //beq
    str = "eqfun" + std::to_string(my_id + 1); //convert to matlab index
    sprob.beq[my_id].init(casadi::external(str, functionLibrary));

    //bineq
    str = "ineqfun" + std::to_string(my_id + 1); //convert to matlab index
    sprob.bineq[my_id].init(casadi::external(str, functionLibrary));

    sprob.read_AA(control_settings.folder_name_sprob, Nagents);
    sprob.read_ublb(control_settings.folder_name_sprob,my_id);

    int nz_ = sprob.A[my_id].cols();
    VectorXd z_ = Eigen::VectorXd::Zero(nz_);

    //H
    sprob.H[my_id].set_arg(0, z_.data(), z_.size());
    sprob.H[my_id].set_arg(1, p.data(), p.size());
    sprob.H[my_id].eval();

    //g
    sprob.g[my_id].set_arg(0, z_.data(), z_.size());
    sprob.g[my_id].set_arg(1, p.data(), p.size());
    sprob.g[my_id].eval();

    //Aeq
    sprob.Aeq[my_id].set_arg(0, z_.data(), z_.size());
    sprob.Aeq[my_id].set_arg(1, p.data(), p.size());
    sprob.Aeq[my_id].eval();

    //Aineq
    sprob.Aineq[my_id].set_arg(0, z_.data(), z_.size());
    sprob.Aineq[my_id].set_arg(1, p.data(), p.size());
    sprob.Aineq[my_id].eval();

    //beq
    sprob.beq[my_id].set_arg(0, z_.data(), z_.size());
    sprob.beq[my_id].set_arg(1, p.data(), p.size());
    sprob.beq[my_id].eval();
    sprob.beq[my_id].res[0] = -sprob.beq[my_id].res[0];

    //bineq
    sprob.bineq[my_id].set_arg(0, z_.data(), z_.size());
    sprob.bineq[my_id].set_arg(1, p.data(), p.size());
    sprob.bineq[my_id].eval();
    sprob.bineq[my_id].res[0] = -sprob.bineq[my_id].res[0];

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
        v_in[j].gam = VectorXd::Zero(nij); 
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
        v_out[j].gam = VectorXd::Zero(0); 
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
    
    v_in_msg_recv_buff.resize(N_in_neighbors); 
    v_out_msg_recv_buff.resize(N_out_neighbors);
    v_in_msg_recv_queue.resize(N_in_neighbors); 
    v_out_msg_recv_queue.resize(N_out_neighbors);
    for(int i = 0; i < N_in_neighbors; i++){
       v_in_msg_recv_queue[i].reserve(10); 
    }
    for(int i = 0; i < N_out_neighbors; i++){
       v_out_msg_recv_queue[i].reserve(10); 
    }

    return;
}

int HolohoverDmpcAdmmNode::solve(unsigned int maxiter_, bool sync_admm)
{  
    for (unsigned int iter = 0; iter < maxiter_; iter++){

        iter_timer.tic();

        //Step 1: local z update
        loc_timer.tic();
        g_bar = sprob.g[my_id].res[0] + gam - rho*zbar;

        //PIQP
        loc_prob.update(piqp::nullopt, g_bar, piqp::nullopt , sprob.beq[my_id].res[0] , piqp::nullopt , piqp::nullopt, piqp::nullopt, piqp::nullopt,true);
        loc_prob.solve();
        z = loc_prob.result().x;

        for (int i = 0; i < N_og; i++){
            auto idx = og_idx_to_idx.find(i);
            XV[i].clear();
            XV[i].push_back(z(idx->second));
            XV[i].push_back(gam(idx->second) / rho); 
        }
        loc_timer.toc();

        //Communication
        z_comm_timer.tic();
        update_v_in();
        bool z_async_ = send_vin_receive_vout(sync_admm);
        z_comm_timer.toc();
        if(mpc_step_since_log < z_async.rows() && iter < z_async.cols()){
            z_async(mpc_step_since_log,iter) = (int) z_async_;
        }

        //Step 2: averaging
        double avg;
        for (int i = 0; i < N_og; i++){
            avg = std::accumulate(XV[i].begin(), XV[i].end(), 0.0) / (0.5*XV[i].size()); //multiply the denominator by 0.5, because XV includes entries from the primal variable z and from the dual variable 
            auto idx = og_idx_to_idx.find(i);
            zbar[idx->second] = avg;
        }

        //Communication
        zbar_comm_timer.tic();
        update_v_out();
        bool zbar_async_ = send_vout_receive_vin(sync_admm);
        zbar_comm_timer.toc();
        if(mpc_step_since_log < zbar_async.rows() && iter < zbar_async.cols()){
            zbar_async(mpc_step_since_log,iter) = (int) zbar_async_;
        } 

        // Step 3: dual update
        gam = gam + rho*(z-zbar);  
        iter_timer.toc();

    }     
    
    return 0;
}

void HolohoverDmpcAdmmNode::received_vin_callback(const holohover_msgs::msg::HolohoverADMMStamped &v_in_msg_, int in_neighbor_idx_){
    
    {
        std::lock_guard<std::mutex> lock(v_in_mutex);
        if(v_in_msg_recv_queue[in_neighbor_idx_].size()>=10){
            v_in_msg_recv_queue[in_neighbor_idx_].erase(v_in_msg_recv_queue[in_neighbor_idx_].begin());
        }
        v_in_msg_recv_queue[in_neighbor_idx_].push_back(v_in_msg_);
    }
    v_in_cv.notify_all();

}

void HolohoverDmpcAdmmNode::received_vout_callback(const holohover_msgs::msg::HolohoverADMMStamped &v_out_msg_, int out_neighbor_idx_){
    
    {
        std::lock_guard<std::mutex> lock(v_out_mutex);
        if(v_out_msg_recv_queue[out_neighbor_idx_].size()>=10){
            v_out_msg_recv_queue[out_neighbor_idx_].erase(v_out_msg_recv_queue[out_neighbor_idx_].begin());
        }
        v_out_msg_recv_queue[out_neighbor_idx_].push_back(v_out_msg_);
    }
    v_out_cv.notify_all();
   
} 

bool HolohoverDmpcAdmmNode::send_vin_receive_vout(bool sync_admm){
    //send copies in vin and receive copies in vout
    //this is done before the averaging step
    bool is_async = false;

    //send vin
    send_vin_timer.tic();
    for (int i = 0; i < N_in_neighbors; i++){
        v_in_msg[i].seq_number += 1;
        v_in_msg[i].header.frame_id = "body"; 
        v_in_msg[i].header.stamp = this->now(); 
        Eigen::VectorXf::Map(&v_in_msg[i].value[0], v_in[i].val.size()) = v_in[i].val.cast<float>();
        Eigen::VectorXf::Map(&v_in_msg[i].gamma[0], v_in[i].gam.size()) = v_in[i].gam.cast<float>();
        v_in_publisher[i]->publish(v_in_msg[i]); //ros

    }
    send_vin_timer.toc();
    //receive vout
    receive_vout_timer.tic();
    Eigen::Array<bool,Dynamic,1> received(N_out_neighbors,1);
    received.fill(false);
    
    const std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
    //const std::chrono::steady_clock::time_point t_wake = t_start + std::chrono::milliseconds(5);
    while (!received.all()){
        
        std::unique_lock<std::mutex> lock(v_out_mutex);

        for (int i = 0; i < N_out_neighbors; i++){
            if (!received(i)){
                //v_out_lock[i].lock();
                for (auto it = v_out_msg_recv_queue[i].begin(); it != v_out_msg_recv_queue[i].end();) {
                    if (it->seq_number < v_in_msg[i].seq_number) {
                        RCLCPP_INFO(get_logger(), "receive vout: sent seq_number %ld and received seq_number %ld from neighbor %d", v_in_msg[i].seq_number, it->seq_number, i);
                        it = v_out_msg_recv_queue[i].erase(it);
                    } else if (it->seq_number == v_in_msg[i].seq_number) {
                        v_out_msg_recv_buff[i] = *it;
                        it = v_out_msg_recv_queue[i].erase(it);
                        received(i) = true;
                        break;
                    } else {
                        RCLCPP_INFO(get_logger(), "receive vout: sent seq_number %ld and received seq_number %ld from neighbor %d", v_in_msg[i].seq_number, it->seq_number, i);
                        ++it;
                    }
                }
                
            }
        }
        if(!sync_admm){ 
            std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
            long duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
            long duration_since_mpc_step_start = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - convert_u_acc_timer.m_tic).count();
            if (duration_ms > control_settings.max_wait_time*1000 || duration_since_mpc_step_start > control_settings.max_wait_fraction*control_settings.dmpc_period*1000){
                is_async = true;
                RCLCPP_INFO(get_logger(), "proceeding ADMM without having received all v_out messages");
                break;
            }            
        }
        if (!received.all()){
            const std::chrono::steady_clock::time_point t_wake = t_start + std::chrono::milliseconds(5);
            v_out_cv.wait_until(lock,t_wake);
        }
    }
    
    //load last received messages into XV
        for (int i = 0; i < N_out_neighbors; i++){
                int idx = 0;
                for (int j = 0; j < v_out_msg_recv_buff[i].val_length; j++){
                    idx = v_out_msg_idx_first_received[i][j];
                    auto og_idx = idx_to_og_idx.find(idx);
                    if (og_idx != idx_to_og_idx.end()){
                        XV[og_idx->second].push_back(v_out_msg_recv_buff[i].value[j]); //careful: the order in XV will depend on the order in which the messages arrive. But that does not affect the average value.
                        XV[og_idx->second].push_back(v_out_msg_recv_buff[i].gamma[j] / rho);
                    }
                } 
        }  
    receive_vout_timer.toc();


    return is_async;
}

bool HolohoverDmpcAdmmNode::send_vout_receive_vin(bool sync_admm){
    //send originals in vout and receive copies in vin
    //this is done after the averaging step
    bool is_async = false;

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
    const std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
    //const std::chrono::steady_clock::time_point t_wake = t_start + std::chrono::milliseconds(5);
    while (!received.all()){
        
        std::unique_lock<std::mutex> lock(v_in_mutex);

        for (int i = 0; i < N_in_neighbors; i++){
            if (!received(i)){
                for (auto it = v_in_msg_recv_queue[i].begin(); it != v_in_msg_recv_queue[i].end();) {
                    if (it->seq_number < v_out_msg[i].seq_number) {
                        RCLCPP_INFO(get_logger(), "receive vin: sent seq_number %ld and received seq_number %ld from neighbor %d", v_out_msg[i].seq_number, it->seq_number, i);
                        it = v_in_msg_recv_queue[i].erase(it);
                    } else if (it->seq_number == v_out_msg[i].seq_number) {
                        v_in_msg_recv_buff[i] = *it;
                        it = v_in_msg_recv_queue[i].erase(it);
                        received(i) = true;
                        for (int j = 0; j < v_in_msg_recv_buff[i].val_length; j++){
                                og_idx = v_in_msg_idx_first_received[i][j];
                                auto tmp = v_in[i].og_idx_to_cpy_idx.find(og_idx);
                                if (tmp != v_in[i].og_idx_to_cpy_idx.end()){
                                cpy_idx = tmp->second;
                                zbar[cpy_idx] = v_in_msg_recv_buff[i].value[j];
                            }
                        }
                        break;
                    } else {
                        RCLCPP_INFO(get_logger(), "receive vin: sent seq_number %ld and received seq_number %ld from neighbor %d", v_out_msg[i].seq_number, it->seq_number, i);
                        ++it;
                    }
                }
            }
        }
        if(!sync_admm){ 
            std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
            long duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
            long duration_since_mpc_step_start = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - convert_u_acc_timer.m_tic).count();
            if (duration_ms > control_settings.max_wait_time*1000 || duration_since_mpc_step_start > control_settings.max_wait_fraction*control_settings.dmpc_period*1000 ){
                RCLCPP_INFO(get_logger(), "proceeding ADMM without having received all v_in messages");
                is_async = true;
                break;
            }
        }
        if (!received.all()){
            const std::chrono::steady_clock::time_point t_wake = t_start + std::chrono::milliseconds(5);
            v_in_cv.wait_until(lock,t_wake);
        }       
    }

    return is_async;
}


void HolohoverDmpcAdmmNode::update_v_in(){
    //update all v_in values with values from z
    for (int i = 0; i < N_in_neighbors; i++){
        for (int j = 0; j < v_in[i].nv; j++){
            v_in[i].val[j] = z[v_in[i].cpy_idx[j]];
            v_in[i].gam[j] = gam[v_in[i].cpy_idx[j]];   
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
            QUILL_LOG_INFO(quill_logger, "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}", logged_mpc_steps+k, x_log(k,0), x_log(k,1), x_log(k,2), x_log(k,3), x_log(k,4), x_log(k,5), u_log(k,0), u_before_conversion_log(k,1), u_before_conversion_log(k,2), u_before_conversion_log(k,0), u_log(k,1), u_log(k,2), xd_log(k,0), xd_log(k,1), xd_log(k,2), xd_log(k,3), xd_log(k,4), xd_log(k,5), ud_log(k,0), ud_log(k,1), ud_log(k,2), get_state_timer.m_log[k], convert_u_acc_timer.m_log[k], publish_signal_timer.m_log[k], update_setpoint_timer.m_log[k], admm_timer.m_log[k], i, iter_timer.m_log[row], loc_timer.m_log[row], z_comm_timer.m_log[row], zbar_comm_timer.m_log[row], send_vin_timer.m_log[row], receive_vout_timer.m_log[row], z_async(k,i), zbar_async(k,i));

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
    convert_u_acc_timer.clear();
    publish_signal_timer.clear();
    update_setpoint_timer.clear();
    get_state_timer.clear();
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