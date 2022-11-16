#include "control_signal_node.hpp"
#include <cmath>

HolohoverControlSignalNode::HolohoverControlSignalNode() :
        Node("control_signal", rclcpp::NodeOptions().allow_undeclared_parameters(true)
                                                 .automatically_declare_parameters_from_overrides(true)),
        holohover_props(load_holohover_pros(*this)),
        control_settings(load_control_lqr_settings(*this)),
        holohover(holohover_props, control_settings.period)
{
    // init state
    state.setZero();

    // init ref
    ref.x = 0;
    ref.y = 0;
    ref.theta = 0;

    init_topics();
    init_timer();
}

void HolohoverControlSignalNode::init_topics()
{
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControl>(
            "drone/control",
            rclcpp::SensorDataQoS());

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverState>(
            "navigation/state", 10,
            std::bind(&HolohoverControlSignalNode::state_callback, this, std::placeholders::_1));
    reference_subscription = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "control/ref", 10,
            std::bind(&HolohoverControlSignalNode::ref_callback, this, std::placeholders::_1));
}

void HolohoverControlSignalNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(control_settings.period),
            std::bind(&HolohoverControlSignalNode::publish_control, this));
}

void HolohoverControlSignalNode::publish_control()
{
    Holohover::state_t<double> state_ref;
    state_ref.setZero();
    state_ref(0) = ref.x;
    state_ref(1) = ref.y;
    state_ref(4) = ref.theta;
    
	static int comb = 0;
	
	static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	
	auto us_counter = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

	//std::cout << "Time difference = " << us_counter << "[µs]" << std::endl;
	//std::cout << "counter = " << counter << "\n";
	//std::cout << "comb = " << comb << "\n";
	
    // output signal
    static Holohover::control_force_t<double> u_signal;
    static bool u_signal_init = false;
    
    if(!u_signal_init) {
    	for(int i=0; i<6; i++) {
			u_signal(i) = 0.0;
		}
    	u_signal_init = true;
    }
	  
    if(us_counter >= 2500000) {
    	begin = std::chrono::steady_clock::now();
    	comb += 1;
    	motor_combination(u_signal, comb); // convert combination to signal
    	std::cout << "comb = " << comb << "\n";
    } /*else if(us_counter >= 2000000) {
		for(int i=0; i<6; i++) {
			u_signal(i) = 0.0;
		}
	}*/


    // clip between 0 and 1
    u_signal = u_signal.cwiseMax(0).cwiseMin(1);

	// publish control msg
    holohover_msgs::msg::HolohoverControl control_msg;
    control_msg.motor_a_1 = u_signal(0);
    control_msg.motor_a_2 = u_signal(1);
    control_msg.motor_b_1 = u_signal(2);
    control_msg.motor_b_2 = u_signal(3);
    control_msg.motor_c_1 = u_signal(4);
    control_msg.motor_c_2 = u_signal(5);
    control_publisher->publish(control_msg);
}

void HolohoverControlSignalNode::motor_combination(Holohover::control_force_t<double>& u_signal, int comb)
{
	/*int a = comb % NB_SIGNALS;
	float signal_a = a / (NB_SIGNALS-1);
	
	comb = comb/NB_SIGNALS;
	int b = int(std::floor(comb)) % NB_SIGNALS;
	float signal_b = b / (NB_SIGNALS-1);
	
	comb = comb/NB_SIGNALS;
	int c = int(std::floor(comb)) % NB_SIGNALS;
	float signal_c = c / (NB_SIGNALS-1);
	
	comb /= NB_MOTORS;
	int motor_a = int(std::floor(comb)) % NB_MOTORS;
	
	comb /= NB_MOTORS;
	int motor_b = int(std::floor(comb)) % NB_MOTORS;
	
	comb /= NB_MOTORS;
	int motor_c = int(std::floor(comb)) % NB_MOTORS;*/
	
	uint16_t ma = (comb & 0x0001);
	uint16_t mb = (comb & 0x0002) >> 1;
	uint16_t mc = (comb & 0x0004) >> 2;
	uint16_t sa = (comb & 0x0038) >> 3;
	uint16_t sb = (comb & 0x01C0) >> 6;
	uint16_t sc = (comb & 0x0E00) >> 9;
	
	float signal_a = float(sa)/7;
	float signal_b = float(sb)/7;
	float signal_c = float(sc)/7;
	
	int motor_mask[6] = {0,0,0,0,0,0};
	motor_mask[ma] = 1;
	motor_mask[mb+2] = 1;
	motor_mask[mc+4] = 1;
	
	float signal[6] = {signal_a,signal_a,signal_b,signal_b,signal_c,signal_c};
	
	std::cout << "signal = ";
	for (int i=0; i<6; i++) {
		std::cout << signal[i] << ", ";
	}
	std::cout << std::endl;
	
	std::cout << "mask = ";
	for (int i=0; i<6; i++) {
		std::cout << motor_mask[i];
	}
	std::cout << std::endl;
	
	// set ouput signal
	for(int i=0; i<6; i++) {
		u_signal(i) = signal[i] * motor_mask[i];
	}
}

void HolohoverControlSignalNode::state_callback(const holohover_msgs::msg::HolohoverState &state_msg)
{
    state(0) = state_msg.x;
    state(1) = state_msg.y;
    state(2) = state_msg.v_x;
    state(3) = state_msg.v_y;
    state(4) = state_msg.yaw;
    state(5) = state_msg.w_z;
}

void HolohoverControlSignalNode::ref_callback(const geometry_msgs::msg::Pose2D &pose)
{
    ref = pose;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlSignalNode>());
    rclcpp::shutdown();
    return 0;
}
