#include <cmath>
#include "simulation_node.hpp"
#include <tf2/LinearMath/Quaternion.h>

HolohoverSimulationNode::HolohoverSimulationNode() :
    Node("simulation"),
    holohover_props(load_holohover_pros(*this)),
    simulation_settings(load_simulation_settings(*this)),
    holohover(holohover_props, simulation_settings.period),
    random_engine(simulation_settings.seed)
{
    // init state
    state.setZero();
    nonlinear_state.setZero();

    // init zero control
    current_control.header.frame_id = "body";
    current_control.header.stamp = this->now();
    current_control.motor_a_1 = 0;
    current_control.motor_a_2 = 0;
    current_control.motor_b_1 = 0;
    current_control.motor_b_2 = 0;
    current_control.motor_c_1 = 0;
    current_control.motor_c_2 = 0;
    motor_velocities.setZero();
    calculate_control_acc();

    init_topics();
    init_timers();
}

void HolohoverSimulationNode::init_topics()
{
    state_publisher = this->create_publisher<holohover_msgs::msg::HolohoverStateStamped>("simulation/state", 10);
    imu_publisher = this->create_publisher<holohover_msgs::msg::HolohoverIMUStamped>(
            "drone/imu",
            rclcpp::SensorDataQoS());
    mouse_publisher = this->create_publisher<holohover_msgs::msg::HolohoverMouseStamped>(
            "drone/mouse",
            rclcpp::SensorDataQoS());
    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "optitrack/drone/pose",
            rclcpp::SensorDataQoS());

    control_subscription = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(
            "drone/control",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverSimulationNode::control_callback, this, std::placeholders::_1));
}

void HolohoverSimulationNode::init_timers()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(simulation_settings.period),
            std::bind(&HolohoverSimulationNode::simulate_forward_callback, this));

    if (simulation_settings.sensor_pose_period > 0) {
        pose_timer = this->create_wall_timer(
                std::chrono::duration<double>(simulation_settings.sensor_pose_period),
                std::bind(&HolohoverSimulationNode::pose_callback, this));
    }
}

void HolohoverSimulationNode::calculate_control_acc()
{
    Holohover::control_force_t<double> current_control_force;
    holohover.signal_to_thrust(motor_velocities, current_control_force);
    holohover.control_force_to_acceleration(state, current_control_force, current_control_acc);

    // add drag force
    double rho = 1.2; // air density at ~20C sea level
    current_control_acc(0) -= 0.5 / holohover_props.mass * rho * state(2) * abs(state(2)) * simulation_settings.drag_coefficient * simulation_settings.drag_reference_area;
    current_control_acc(1) -= 0.5 / holohover_props.mass * rho * state(3) * abs(state(3)) * simulation_settings.drag_coefficient * simulation_settings.drag_reference_area;

    // add force introduced by table tilt
    double g = 9.81;
    current_control_acc(0) += sin(simulation_settings.table_tilt_x * M_PI / 180.0) * g;
    current_control_acc(1) += sin(simulation_settings.table_tilt_y * M_PI / 180.0) * g;

    //std::cout << "acc = " << current_control_acc.transpose() << std::endl;
    //std::cout << "force = " << current_control_force.transpose() << std::endl;
}

void HolohoverSimulationNode::simulate_forward_callback()
{
    // integrate motor velocities
    Holohover::control_force_t<double> current_control_signal;
    current_control_signal(0) = current_control.motor_a_1;
    current_control_signal(1) = current_control.motor_a_2;
    current_control_signal(2) = current_control.motor_b_1;
    current_control_signal(3) = current_control.motor_b_2;
    current_control_signal(4) = current_control.motor_c_1;
    current_control_signal(5) = current_control.motor_c_2;
    motor_velocities = holohover.Ad_motor * motor_velocities + holohover.Bd_motor * current_control_signal;
    calculate_control_acc();

    state = holohover.Ad * state + holohover.Bd * current_control_acc;

    holohover.template non_linear_state_dynamics_discrete<double>(nonlinear_state, motor_velocities, nonlinear_state);
    //state = nonlinear_state;
    // std::cout << "models difference = " << (state-nonlinear_state) << std::endl;
    // std::cout << "state = " << (state) << std::endl;
    // std::cout << "nonlinear_state = " << (nonlinear_state) << std::endl;
    // std::cout << "state- nonlinear_state = " << (state-nonlinear_state) << std::endl;
    // std::cout << "norm of models difference = " << (state-nonlinear_state).norm() << std::endl;


    holohover_msgs::msg::HolohoverStateStamped msg_state;
    msg_state.header.frame_id = "world";
    msg_state.header.stamp = this->now();
    msg_state.state_msg.x = state(0)-holohover_props.CoM[0];
    msg_state.state_msg.y = state(1)-holohover_props.CoM[1];
    msg_state.state_msg.v_x = state(2);
    msg_state.state_msg.v_y = state(3);
    msg_state.state_msg.yaw = state(4);
    msg_state.state_msg.w_z = state(5);

    if (msg_state.state_msg.yaw < -M_PI)
    {
        msg_state.state_msg.yaw += 2 * M_PI;
    }
    if (msg_state.state_msg.yaw > M_PI)
    {
        msg_state.state_msg.yaw -= 2 * M_PI;
    }

    state_publisher->publish(msg_state);
}


void HolohoverSimulationNode::pose_callback()
{
    geometry_msgs::msg::PoseStamped pose_measurement;
    pose_measurement.header.frame_id = "world";
    pose_measurement.header.stamp = this->now();
    pose_measurement.pose.position.x = state(0) + simulation_settings.Gx;
    pose_measurement.pose.position.y = state(1) + simulation_settings.Gy;
    pose_measurement.pose.position.z = 0;
    double theta = state(4);

    std::normal_distribution<> pose_x_noise(0, simulation_settings.sensor_pose_noise_x);
    std::normal_distribution<> pose_y_noise(0, simulation_settings.sensor_pose_noise_y);
    std::normal_distribution<> pose_yaw_noise(0, simulation_settings.sensor_pose_noise_yaw);
    pose_measurement.pose.position.x += pose_x_noise(random_engine);
    pose_measurement.pose.position.y += pose_y_noise(random_engine);
    theta += pose_yaw_noise(random_engine);

    if (theta < -M_PI)
    {
        theta += 2 * M_PI;
    }
    if (theta > M_PI)
    {
        theta -= 2 * M_PI;
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    pose_measurement.pose.orientation.w = q.w();
    pose_measurement.pose.orientation.x = q.x();
    pose_measurement.pose.orientation.y = q.y();
    pose_measurement.pose.orientation.z = q.z();

    pose_publisher->publish(pose_measurement);
}

void HolohoverSimulationNode::control_callback(const holohover_msgs::msg::HolohoverControlStamped &control)
{
    current_control = control;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverSimulationNode>());
    rclcpp::shutdown();
    return 0;
}
