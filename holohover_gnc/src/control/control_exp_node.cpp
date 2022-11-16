#include "control_lqr_node.hpp"

HolohoverControlLQRNode::HolohoverControlLQRNode() :
        Node("control_lqr", rclcpp::NodeOptions().allow_undeclared_parameters(true)
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

void HolohoverControlLQRNode::init_topics()
{
    control_publisher = this->create_publisher<holohover_msgs::msg::HolohoverControl>(
            "drone/control",
            rclcpp::SensorDataQoS());

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverState>(
            "navigation/state", 10,
            std::bind(&HolohoverControlLQRNode::state_callback, this, std::placeholders::_1));
    reference_subscription = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "control/ref", 10,
            std::bind(&HolohoverControlLQRNode::ref_callback, this, std::placeholders::_1));
}

void HolohoverControlLQRNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(control_settings.period),
            std::bind(&HolohoverControlLQRNode::publish_control, this));
}

void HolohoverControlLQRNode::publish_control()
{
    Holohover::state_t<double> state_ref;
    state_ref.setZero();
    state_ref(0) = ref.x;
    state_ref(1) = ref.y;
    state_ref(4) = ref.theta;
    
    // added by Nicolaj
    /*Holohover::state_t<double> state_rand;
    random_state(state_rand);
    Holohover::control_acc_t<double> u_acc_rand;
    random_control_acc(u_acc_rand);*/
    
    // added by Nicolaj
    static Holohover::state_t<double> vel_rand;
    rand_vel(vel_rand, state);

    // modified by Nicolaj
    //Holohover::control_acc_t<double> u_acc = -K * (state - state_ref + state_rand) + u_acc_rand;
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

void HolohoverControlLQRNode::state_callback(const holohover_msgs::msg::HolohoverState &state_msg)
{
    state(0) = state_msg.x;
    state(1) = state_msg.y;
    state(2) = state_msg.v_x;
    state(3) = state_msg.v_y;
    state(4) = state_msg.yaw;
    state(5) = state_msg.w_z;
}

void HolohoverControlLQRNode::ref_callback(const geometry_msgs::msg::Pose2D &pose)
{
    ref = pose;
}

/*
* Added by Nicolaj
*/
void HolohoverControlLQRNode::rand_vel(Holohover::state_t<double>& vel_rand, 
				       const Holohover::state_t<double>& state)
{

    /*static auto time_before = std::chrono::high_resolution_clock::now();
    
    auto time_now = std::chrono::high_resolution_clock::now();
    auto time_duration = std::chrono::duration_cast<std::chrono::microseconds>(time_before - time_now);
    double update_time = time_duration.count() / 1000000;
    time_before = time_now;*/
    float periode = 0.01;
    static int counter = 300;
    
    // calc. next position in fct. of current one and velocity
    vel_rand(0,0) = state(0,0) + vel_rand(2,0)*periode;
    vel_rand(1,0) = state(1,0) + vel_rand(3,0)*periode;
    vel_rand(4,0) = state(4,0) + vel_rand(5,0)*periode;
    
    counter -= 1;    
    if(counter <= 0) {
        vel_rand(2,0) = rand()*(RAND_VEL_MAX-RAND_VEL_MIN)/RAND_MAX + RAND_VEL_MIN;
        vel_rand(3,0) = rand()*(RAND_VEL_MAX-RAND_VEL_MIN)/RAND_MAX + RAND_VEL_MIN;
        vel_rand(5,0) = rand()*(RAND_OMG_MAX-RAND_OMG_MIN)/RAND_MAX + RAND_OMG_MIN;   
        counter = rand()*(RAND_SIG_MAX-RAND_SIG_MIN)/RAND_MAX + RAND_SIG_MIN;;       
    }

    if( (state(0,0)>RAND_POS_MAX && vel_rand(2,0)>0) 
        || (state(0,0)<RAND_POS_MIN && vel_rand(2,0)<0) ) {
        vel_rand(2,0) = -vel_rand(2,0);
    }    
    if( (state(1,0)>RAND_POS_MAX && vel_rand(3,0)>0) 
        || (state(1,0)<RAND_POS_MIN && vel_rand(3,0)<0) ) {
        vel_rand(3,0) = -vel_rand(3,0);
    }
    if( (state(4,0)>RAND_ANG_MAX && vel_rand(5,0)>0) 
        || (state(4,0)<RAND_ANG_MIN && vel_rand(5,0)<0) ) {
        vel_rand(5,0) = -vel_rand(5,0);
    }  
    
    /*
    static bool not_init = true;
    if(not_init) {
        vel_rand(2,0) = 0.3;
        not_init = false;
    }
    

    // if drone reached x border, choose a new velocity in x
    if(state(0,0)>RAND_POS_MAX || state(0,0)<RAND_POS_MIN) {
        if(state(0,0)>RAND_POS_MAX) {
            vel_rand(2,0) = rand()*RAND_VEL_MIN/RAND_MAX;
        } else {
            vel_rand(2,0) = rand()*RAND_VEL_MAX/RAND_MAX;
        }
        vel_rand(3,0) = rand()*(RAND_VEL_MAX-RAND_VEL_MIN)/RAND_MAX + RAND_VEL_MIN;
        vel_rand(5,0) = rand()*(RAND_OMG_MAX-RAND_OMG_MIN)/RAND_MAX + RAND_OMG_MIN;    
    
    }
    
    // if drone reached y border, choose a new velocity in y
    if(state(1,0)>RAND_POS_MAX || state(1,0)<RAND_POS_MIN) {
        if(state(1,0)>RAND_POS_MAX) {
            vel_rand(3,0) = rand()*RAND_VEL_MIN/RAND_MAX;
        } else {
            vel_rand(3,0) = rand()*RAND_VEL_MAX/RAND_MAX;
        }
        vel_rand(2,0) = rand()*(RAND_VEL_MAX-RAND_VEL_MIN)/RAND_MAX + RAND_VEL_MIN;
        vel_rand(5,0) = rand()*(RAND_OMG_MAX-RAND_OMG_MIN)/RAND_MAX + RAND_OMG_MIN;         
    }
    
    // calc. next position in fct. of current one and velocity
    vel_rand(0,0) = state(0,0) + vel_rand(2,0)*periode;
    vel_rand(1,0) = state(1,0) + vel_rand(3,0)*periode;
    vel_rand(4,0) = state(4,0) + vel_rand(5,0)*periode;*/

}

/*
* Added by Nicolaj
*/
void HolohoverControlLQRNode::random_state(Holohover::state_t<double>& state_rand)
{
    static int signal_length = 0;
    static int counter = 0;
    
    if(counter<signal_length) {
    	counter++;
    	return;
    }
    
    // choose random state for x, y and angle
    state_rand(0,0) = rand()*(RAND_POS_MAX-RAND_POS_MIN)/RAND_MAX + RAND_POS_MIN;
    state_rand(1,0) = rand()*(RAND_POS_MAX-RAND_POS_MIN)/RAND_MAX + RAND_POS_MIN;
    state_rand(4,0) = rand()*(RAND_ANG_MAX-RAND_ANG_MIN)/RAND_MAX + RAND_ANG_MIN;

    // choose random length of signal
    signal_length = rand()*(RAND_STATE_SIG_MAX-RAND_STATE_SIG_MIN)/RAND_MAX + RAND_STATE_SIG_MIN;

    // reset counter
    counter = 0;
}

/*
* Added by Nicolaj
*/
void HolohoverControlLQRNode::random_control_acc(Holohover::control_acc_t<double>& u_acc_rand)
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
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverControlLQRNode>());
    rclcpp::shutdown();
    return 0;
}
