#include "navigation_node.hpp"

HolohoverNavigationNode::HolohoverNavigationNode() :
    Node("navigation", rclcpp::NodeOptions().allow_undeclared_parameters(true)
                                            .automatically_declare_parameters_from_overrides(true)),
    holohover_props(load_holohover_pros(*this)),
    navigation_settings(load_navigation_settings(*this)),
    holohover(holohover_props, navigation_settings.period),
    kalman(holohover, navigation_settings)
{
    // init zero control
    current_control.motor_a_1 = 0;
    current_control.motor_a_2 = 0;
    current_control.motor_b_1 = 0;
    current_control.motor_b_2 = 0;
    current_control.motor_c_1 = 0;
    current_control.motor_c_2 = 0;

    init_topics();
    init_timer();
}

void HolohoverNavigationNode::init_topics()
{
    state_publisher = this->create_publisher<holohover_msgs::msg::HolohoverState>("navigation/state", 10);
    control_accel_publisher = this->create_publisher<geometry_msgs::msg::Accel>("navigation/control_accel", 10);
    comp_time_publisher = this->create_publisher<std_msgs::msg::Float64>("navigation/debug/comp_time", 10);

    imu_subscription = this->create_subscription<holohover_msgs::msg::HolohoverIMU>(
            "drone/imu",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverNavigationNode::imu_callback, this, std::placeholders::_1));
    mouse_subscription = this->create_subscription<holohover_msgs::msg::HolohoverMouse>(
            "drone/mouse",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverNavigationNode::mouse_callback, this, std::placeholders::_1));
    pose_subscription = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "optitrack/drone/pose",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverNavigationNode::pose_callback, this, std::placeholders::_1));
    control_subscription = this->create_subscription<holohover_msgs::msg::HolohoverControl>(
            "drone/control",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverNavigationNode::control_callback, this, std::placeholders::_1));
}

void HolohoverNavigationNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(navigation_settings.period),
            std::bind(&HolohoverNavigationNode::kalman_predict_step, this));
}

void HolohoverNavigationNode::kalman_predict_step()
{
    rclcpp::Time start_time = this->now();

    HolohoverEKF::sensor_imu_t imu_sensor;
    imu_sensor(0) = current_imu.acc.x;
    imu_sensor(1) = current_imu.acc.y;
    imu_sensor(2) = current_imu.gyro.z;

    HolohoverEKF::control_force_t u_signal;
    HolohoverEKF::control_force_t u_force;
    u_signal(0) = current_control.motor_a_1;
    u_signal(1) = current_control.motor_a_2;
    u_signal(2) = current_control.motor_b_1;
    u_signal(3) = current_control.motor_b_2;
    u_signal(4) = current_control.motor_c_1;
    u_signal(5) = current_control.motor_c_2;
    holohover.signal_to_thrust(u_signal, u_force);

    if (received_control && received_imu)
    {
        kalman.predict_control_force_and_sensor_acc(u_force, imu_sensor);
    }
    else if (received_imu)
    {
        kalman.predict_sensor_acc(imu_sensor);
    }
    else
    {
        kalman.predict_control_force(u_force);
    }

    rclcpp::Time end_time = this->now();

    holohover_msgs::msg::HolohoverState estimated_state;
    estimated_state.x = kalman.x(0);
    estimated_state.y = kalman.x(1);
    estimated_state.v_x = kalman.x(2);
    estimated_state.v_y = kalman.x(3);
    estimated_state.yaw = kalman.x(4);
    estimated_state.w_z = kalman.x(5);

    std_msgs::msg::Float64 comp_time;
    comp_time.data = (end_time - start_time).seconds();

    state_publisher->publish(estimated_state);
    comp_time_publisher->publish(comp_time);
}

void HolohoverNavigationNode::imu_callback(const holohover_msgs::msg::HolohoverIMU &measurement)
{
    current_imu = measurement;
    received_imu = true;
}

void HolohoverNavigationNode::mouse_callback(const holohover_msgs::msg::HolohoverMouse &measurement)
{
    HolohoverEKF::sensor_mouse_t mouse_measurement;
    mouse_measurement(0) = measurement.v_x;
    mouse_measurement(1) = measurement.v_y;
    kalman.update_sensor_mouse(mouse_measurement);
}

void HolohoverNavigationNode::pose_callback(const geometry_msgs::msg::Pose2D &measurement)
{
    HolohoverEKF::sensor_pose_t pose_measurement;
    pose_measurement(0) = measurement.x;
    pose_measurement(1) = measurement.y;
    pose_measurement(2) = measurement.theta;
    kalman.update_sensor_pose(pose_measurement);
}

void HolohoverNavigationNode::control_callback(const holohover_msgs::msg::HolohoverControl &control)
{
    current_control = control;
    received_control = true;

    // convert control signal into control acceleration
    Holohover::control_force_t<double> u_signal;
    u_signal(0) = control.motor_a_1;
    u_signal(1) = control.motor_a_2;
    u_signal(2) = control.motor_b_1;
    u_signal(3) = control.motor_b_2;
    u_signal(4) = control.motor_c_1;
    u_signal(5) = control.motor_c_2;
    Holohover::control_force_t<double> u_force;
    holohover.signal_to_thrust(u_signal, u_force);
    Holohover::control_acc_t<double> u_acc;
    holohover.control_force_to_acceleration(kalman.x, u_force, u_acc);

    // publish control acceleration
    geometry_msgs::msg::Accel control_accel;
    control_accel.linear.x = u_acc(0);
    control_accel.linear.y = u_acc(1);
    control_accel.angular.z = u_acc(2);
    control_accel_publisher->publish(control_accel);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverNavigationNode>());
    rclcpp::shutdown();
    return 0;
}
