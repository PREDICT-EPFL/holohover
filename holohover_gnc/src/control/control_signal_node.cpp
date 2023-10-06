#include "control_signal_node.hpp"
#include <cmath>

HolohoverControlSignalNode::HolohoverControlSignalNode() :
        Node("control_signal"),
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
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControlStamped>(
            "drone/control",
            rclcpp::SensorDataQoS());
}

void HolohoverControlSignalNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(0.01),
            std::bind(&HolohoverControlSignalNode::publish_control, this));
}

void HolohoverControlSignalNode::publish_control()
{
    Holohover::state_t<double> state_ref;
    state_ref.setZero();
    state_ref(0) = ref.x;
    state_ref(1) = ref.y;
    state_ref(4) = ref.theta;
    
	static int comb = 0; // combination (counter) that is converted into a signal
	
	// elapse time measurement
	static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	auto us_counter = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
	
    // output signal
    static Holohover::control_force_t<double> u_signal;
    static bool u_signal_init = false;
    
    // init. u_signal for the first time
    if(!u_signal_init) {
    	for(int i=0; i<NB_MOTORS; i++) {
			u_signal(i) = holohover_props.idle_signal;
		}
    	u_signal_init = true;
    }
    
    // change signal if time is passed
    if(us_counter >= CYCLE_TIME) {
    	begin = std::chrono::steady_clock::now();
    	comb2signal(u_signal, comb); // convert combination to signal
    	comb += 1;
    } else if(us_counter >= (CYCLE_TIME-OFF_TIME)) {
		for(int i=0; i<NB_MOTORS; i++) {
			u_signal(i) = holohover_props.idle_signal;
		}
	}


    // clip between 0 and 1
    u_signal = u_signal.cwiseMax(holohover_props.idle_signal).cwiseMin(1);

	// publish control msg
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

/*
*  Turn one motor after each other on with 16 signals in the range (0,1]
*  Converts the current combination into a signal
*/
void HolohoverControlSignalNode::comb2signal(Holohover::control_force_t<double>& u_signal, int comb)
{
	uint16_t s = (comb & 0x000F); // masking bits 0-3
	uint16_t m = (comb & 0x0070) >> 4; // masking bits 4-6 and shifting them to zero
	
	float signal = (float(s)+1)/NB_SIGNALS;
	
	int motor_mask[NB_MOTORS] = {0,0,0,0,0,0};
	if(m < NB_MOTORS) {
		motor_mask[m] = 1;
	}
	
	// set ouput signal (all motors are always moving at IDLE_SIGNAL)
	for(int i=0; i<NB_MOTORS; i++) {
		u_signal(i) = (signal-holohover_props.idle_signal) * float(motor_mask[i]) + holohover_props.idle_signal;
	}
	
	// print resulting signal and mask
	std::cout << "comb = " << comb << std::endl;
	std::cout << "signal = " << signal << std::endl;	
	std::cout << "mask = ";
	for (int i=0; i<NB_MOTORS; i++) {
		std::cout << motor_mask[i];
	}
	std::cout << std::endl;
	std::cout << "u = [";
	for (int i=0; i<NB_MOTORS; i++) {
		std::cout << u_signal(i) << ", ";
	}
	std::cout << "]" << std::endl;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlSignalNode>());
    rclcpp::shutdown();
    return 0;
}
