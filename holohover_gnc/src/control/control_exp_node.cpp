#include "control_exp_node.hpp"
#include <cmath>

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
    ref.yaw = 0;

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
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControlStamped>(
            "drone/control",
            rclcpp::SensorDataQoS());

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverStateStamped>(
            "navigation/state", 10,
            std::bind(&HolohoverControlExpNode::state_callback, this, std::placeholders::_1));

    reference_subscription = this->create_subscription<holohover_msgs::msg::HolohoverState>(
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
    state_ref(4) = ref.yaw;

	// calc. acc. using LQR controler
    Holohover::control_acc_t<double> u_acc = -K * (state - state_ref); 
    
    // convert acc. to signal of motors
    Holohover::control_force_t<double> u_force;
    holohover.control_acceleration_to_force(state, u_acc, u_force);
    Holohover::control_force_t<double> u_signal;
    holohover.thrust_to_signal(u_force, u_signal);

    // add random signal
    rand_sig(u_signal);

    // clip between 0 and 1
    u_signal = u_signal.cwiseMax(IDLE_SIGNAL).cwiseMin(1);
    

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

void HolohoverControlExpNode::state_callback(const holohover_msgs::msg::HolohoverStateStamped &msg_state)
{
    state(0) = msg_state.state_msg.x;
    state(1) = msg_state.state_msg.y;
    state(2) = msg_state.state_msg.v_x;
    state(3) = msg_state.state_msg.v_y;
    state(4) = msg_state.state_msg.yaw;
    state(5) = msg_state.state_msg.w_z;
}

void HolohoverControlExpNode::ref_callback(const holohover_msgs::msg::HolohoverState &pose)
{
    ref = pose;
}


/*
*	Add random signal to LQR-control-signal st. always only one motor of a motor pair is turned on
*/
void HolohoverControlExpNode::rand_sig(Holohover::control_force_t<double>& u_signal)
{
	// random signal that are added to u_signal
	static float rand_sig_a = 0; // random signal of motor-pair A (motor 1 and 2)
	static float rand_sig_b = 0; // random signal of motor-pair B (motor 3 and 4)
	static float rand_sig_c = 0; // random signal of motor-pair C (motor 5 and 6)
	
	// measure current time and calc. time past since last signal change
	static std::chrono::steady_clock::time_point 
    					begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::chrono::microseconds::rep 
						elapse_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
							
	if(elapse_time > exp_settings.rand_sig_duration) {
		// choose new random signals
		rand_sig_a = rand()*(exp_settings.rand_sig_max-exp_settings.rand_sig_min)/RAND_MAX 
    					+ exp_settings.rand_sig_min;
        rand_sig_b = rand()*(exp_settings.rand_sig_max-exp_settings.rand_sig_min)/RAND_MAX 
        				+ exp_settings.rand_sig_min;
        rand_sig_c = rand()*(exp_settings.rand_sig_max-exp_settings.rand_sig_min)/RAND_MAX 
        				+ exp_settings.rand_sig_min; 
		
		begin = end; // reset timer
	}
	
	float u_sig_a = u_signal(0) - u_signal(1);
	float u_sig_b = u_signal(2) - u_signal(3);
	float u_sig_c = u_signal(4) - u_signal(5);
	
	// add random signals to LQR-controller signal
	// if u_sig_a+rand_sig_a>0 add signal to first motor of motor-pair otherwise to second motor
	if(u_sig_a+rand_sig_a > 0) {
		u_signal(0) = u_sig_a + rand_sig_a;
		u_signal(1) = 0;
	} else {
		u_signal(0) = 0;
		u_signal(1) = -u_sig_a - rand_sig_a;
	}
	if(u_sig_b+rand_sig_b > 0) {
		u_signal(2) = u_sig_b + rand_sig_b;
		u_signal(3) = 0;
	} else {
		u_signal(2) = 0;
		u_signal(3) = -u_sig_b - rand_sig_b;
	}
	if(u_sig_c+rand_sig_c > 0) {
		u_signal(4) = u_sig_c + rand_sig_c;
		u_signal(5) = 0;
	} else {
		u_signal(4) = 0;
		u_signal(5) = -u_sig_c - rand_sig_c;
	}
}

/*
*	Make trajectory defined by eva_pos_x and eva_pos_y
*/
void HolohoverControlExpNode::evaluation_pos(Holohover::state_t<double>& state_ref, 
										const Holohover::state_t<double>& state)
{
	// evaluation position
	static int eva_pos_counter = 0;
	float eva_pos_x[8] = { 0.25, 0.25, 0.0, -0.25, -0.25, -0.25, 0.0, 0.25 };
	float eva_pos_y[8] = { 0.0, 0.25, 0.25, 0.25, 0.0, -0.25, -0.25, -0.25};
		
	// calc. distance to random position			
	float dist = std::sqrt( (state(0)-eva_pos_x[eva_pos_counter])*(state(0)-eva_pos_x[eva_pos_counter]) 
							+ (state(1)-eva_pos_y[eva_pos_counter])*(state(1)-eva_pos_y[eva_pos_counter]) );
							
	// choose new random position if it is reached
	if(dist < EVALUATION_DIST_THR) {
       
        // increase position counter if end is not yet reached
        if(eva_pos_counter < 7) {
        	eva_pos_counter++;
        }
	}
	
	// set evaluation position in reference state
	state_ref(0) = eva_pos_x[eva_pos_counter];
	state_ref(1) = eva_pos_y[eva_pos_counter];
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlExpNode>());
    rclcpp::shutdown();
    return 0;
}
