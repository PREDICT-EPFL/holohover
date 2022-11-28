#include "control_exp_node.hpp"

HolohoverControlExpNode::HolohoverControlExpNode() :
        Node("control_lqr", rclcpp::NodeOptions().allow_undeclared_parameters(true)
                                                 .automatically_declare_parameters_from_overrides(true)),
        holohover_props(load_holohover_pros(*this)),
        control_settings(load_control_lqr_settings(*this)),
        exp_settings(load_control_exp_settings(*this)),
        holohover(holohover_props, control_settings.period)
{
    // init state
    state.setZero();

    // init ref
    ref.x = 0;
    ref.y = 0;
    ref.theta = 0;

    // calculate LQR gain
    Eigen::Matrix<double, Holohover::NX, Holohover::NX> &Ad = holohover.Ad;
    Eigen::Matrix<double, Holohover::NX, Holohover::NA> &Bd = holohover.Bd;

    Eigen::Matrix<double, Holohover::NX, Holohover::NX> Q;
    Q.setZero();
    Q.diagonal() << control_settings.weight_x, control_settings.weight_y,
                    control_settings.weight_v_x, control_settings.weight_v_y,
                    control_settings.weight_yaw, control_settings.weight_w_z;

    Eigen::Matrix<double, Holohover::NA, Holohover::NA> R;
    R.setZero();
    R.diagonal() << control_settings.weight_a_x, control_settings.weight_a_y, control_settings.weight_w_dot_z;

    Eigen::Matrix<double, Holohover::NX, Holohover::NX> P;
    solve_riccati_iteration_discrete(Ad, Bd, Q, R, P);

    K = (R + Bd.transpose() * P * Bd).ldlt().solve(Bd.transpose() * P * Ad);

    init_topics();
    init_timer();
}

void HolohoverControlExpNode::init_topics()
{
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControl>(
            "drone/control",
            rclcpp::SensorDataQoS());

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverState>(
            "navigation/state", 10,
            std::bind(&HolohoverControlExpNode::state_callback, this, std::placeholders::_1));
    reference_subscription = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "control/ref", 10,
            std::bind(&HolohoverControlExpNode::ref_callback, this, std::placeholders::_1));
}

void HolohoverControlExpNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(control_settings.period),
            std::bind(&HolohoverControlExpNode::publish_control, this));
}

void HolohoverControlExpNode::publish_control()
{
    Holohover::state_t<double> state_ref;
    state_ref.setZero();
    state_ref(0) = ref.x;
    state_ref(1) = ref.y;
    state_ref(4) = ref.theta;
    
    static Holohover::state_t<double> vel_rand;
    rand_vel(vel_rand, state);

    Holohover::control_acc_t<double> u_acc = -K * (state - vel_rand); 
    Holohover::control_force_t<double> u_force;
    holohover.control_acceleration_to_force(state, u_acc, u_force);
    Holohover::control_force_t<double> u_signal;
    holohover.thrust_to_signal(u_force, u_signal);

    // clip between 0 and 1
    u_signal = u_signal.cwiseMax(0).cwiseMin(1);

    holohover_msgs::msg::HolohoverControl control_msg;
    control_msg.motor_a_1 = u_signal(0);
    control_msg.motor_a_2 = u_signal(1);
    control_msg.motor_b_1 = u_signal(2);
    control_msg.motor_b_2 = u_signal(3);
    control_msg.motor_c_1 = u_signal(4);
    control_msg.motor_c_2 = u_signal(5);
    control_publisher->publish(control_msg);
}

void HolohoverControlExpNode::state_callback(const holohover_msgs::msg::HolohoverState &state_msg)
{
    state(0) = state_msg.x;
    state(1) = state_msg.y;
    state(2) = state_msg.v_x;
    state(3) = state_msg.v_y;
    state(4) = state_msg.yaw;
    state(5) = state_msg.w_z;
}

void HolohoverControlExpNode::ref_callback(const geometry_msgs::msg::Pose2D &pose)
{
    ref = pose;
}

void HolohoverControlExpNode::rand_vel(Holohover::state_t<double>& vel_rand, 
				       const Holohover::state_t<double>& state)
{
	// static time variables
    static int us_cycle_time = 3000000; // amount of time same signal is applied, in us
    static std::chrono::microseconds::rep us_counter = 0; // amount of time signal is already applied, in us
    static std::chrono::steady_clock::time_point 
    						begin = std::chrono::steady_clock::now(); // amount of time since last fct. call
    						
    // measure current time and calc. time past since last fct. call in us
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::chrono::microseconds::rep periode = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
	
	// update time variables
	begin = end;
	us_counter += periode;
	
	//std::cout << "periode = " << periode << ", \t";
	//std::cout << "us_counter = " << us_counter << ", \t";
	//std::cout << "us_cycle_time = " << us_cycle_time << std::endl;
	
    
    // calc. next position in fct. of current one and velocity
    vel_rand(0,0) = state(0,0) + vel_rand(2,0)*periode/1000000;
    vel_rand(1,0) = state(1,0) + vel_rand(3,0)*periode/1000000;
    vel_rand(4,0) = state(4,0) + vel_rand(5,0)*periode/1000000;
    
    
    if(us_counter >= us_cycle_time) {  
    	// choose random velocity	
    	vel_rand(2,0) = rand()*(exp_settings.rand_vel_max-exp_settings.rand_vel_min)/RAND_MAX 
    					+ exp_settings.rand_vel_min;
        vel_rand(3,0) = rand()*(exp_settings.rand_vel_max-exp_settings.rand_vel_min)/RAND_MAX 
        				+ exp_settings.rand_vel_min;
        vel_rand(5,0) = rand()*(exp_settings.rand_omg_max-exp_settings.rand_omg_min)/RAND_MAX 
        				+ exp_settings.rand_omg_min; 
        
        // choose random amount of time to apply signal and reset counter  
        us_cycle_time = rand()*(exp_settings.rand_sig_max-exp_settings.rand_sig_min)/RAND_MAX 
        				+ exp_settings.rand_sig_min; 
        us_counter = 0;
    }

	// reverse velocity if holohover reached limits (and the velocity is not already reverted)
    if( (state(0,0)>exp_settings.rand_pos_max && vel_rand(2,0)>0) 
        || (state(0,0)<exp_settings.rand_pos_min && vel_rand(2,0)<0) ) {
        vel_rand(2,0) = -vel_rand(2,0);
    }    
    if( (state(1,0)>exp_settings.rand_pos_max && vel_rand(3,0)>0) 
        || (state(1,0)<exp_settings.rand_pos_min && vel_rand(3,0)<0) ) {
        vel_rand(3,0) = -vel_rand(3,0);
    }
    if( (state(4,0)>exp_settings.rand_ang_max && vel_rand(5,0)>0) 
        || (state(4,0)<exp_settings.rand_ang_min && vel_rand(5,0)<0) ) {
        vel_rand(5,0) = -vel_rand(5,0);
    } 
    std::cout << "vel = (" << state(2,0) << ", " << state(3,0) << ", " << state(5,0) << std::endl;
    //std::cout << "check: " << exp_settings.rand_pos_max << std::endl;
    //std::cout << "check2: " << control_settings.weight_w_z << std::endl;   
}


/*void HolohoverControlExpNode::random_control_acc(Holohover::control_acc_t<double>& u_acc_rand)
{
    static int signal_length = 0;
    static int counter = 0;
    
    if(counter<signal_length) {
    	counter++;
    	return;
    }
    
    // choose random acceleration command
    for(int i=0; i<u_acc_rand.rows(); i++) {		
         u_acc_rand(i,0) = rand()*(RAND_ACC_MAX-RAND_ACC_MIN)/RAND_MAX + RAND_ACC_MIN;
    }

    // choose random length of signal
    signal_length = rand()*(RAND_ACC_SIG_MAX-RAND_ACC_SIG_MIN)/RAND_MAX + RAND_ACC_SIG_MIN;

    // reset counter
    counter = 0;
}*/

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlExpNode>());
    rclcpp::shutdown();
    return 0;
}
