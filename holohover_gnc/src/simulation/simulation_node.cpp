#include "simulation_node.hpp"

HolohoverSimulationNode::HolohoverSimulationNode() :
    Node("simulation", rclcpp::NodeOptions().allow_undeclared_parameters(true)
                                            .automatically_declare_parameters_from_overrides(true)),
    holohover_props(load_holohover_pros(*this)),
    simulation_settings(load_simulation_settings(*this)),
    holohover(holohover_props, simulation_settings.period),
    random_engine(simulation_settings.seed)
{
    // init state
    state.setZero();

    // init zero control
    current_control.header.frame_id = "body";
    current_control.header.stamp = this->now();
    current_control.motor_a_1 = 0;
    current_control.motor_a_2 = 0;
    current_control.motor_b_1 = 0;
    current_control.motor_b_2 = 0;
    current_control.motor_c_1 = 0;
    current_control.motor_c_2 = 0;
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
    pose_publisher = this->create_publisher<geometry_msgs::msg::Pose2D>(
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
    if (simulation_settings.sensor_imu_period > 0)
    {
        imu_timer = this->create_wall_timer(
                std::chrono::duration<double>(simulation_settings.sensor_imu_period),
                std::bind(&HolohoverSimulationNode::imu_callback, this));
    }
    if (simulation_settings.sensor_mouse_period > 0)
    {
        mouse_timer = this->create_wall_timer(
                std::chrono::duration<double>(simulation_settings.sensor_mouse_period),
                std::bind(&HolohoverSimulationNode::mouse_callback, this));
    }
    if (simulation_settings.sensor_pose_period > 0)
    {
        pose_timer = this->create_wall_timer(
                std::chrono::duration<double>(simulation_settings.sensor_pose_period),
                std::bind(&HolohoverSimulationNode::pose_callback, this));
    }
}

void HolohoverSimulationNode::calculate_control_acc()
{
    Holohover::control_force_t<double> current_control_signal;
    current_control_signal(0) = current_control.motor_a_1;
    current_control_signal(1) = current_control.motor_a_2;
    current_control_signal(2) = current_control.motor_b_1;
    current_control_signal(3) = current_control.motor_b_2;
    current_control_signal(4) = current_control.motor_c_1;
    current_control_signal(5) = current_control.motor_c_2;
    Holohover::control_force_t<double> current_control_force;
    holohover.signal_to_thrust(current_control_signal, current_control_force);
    holohover.control_force_to_acceleration(state, current_control_force, current_control_acc);

    // add drag force
    double rho = 1.2; // air density at ~20C sea level
    current_control_acc(0) -= 0.5 / holohover_props.mass * rho * state(2) * abs(state(2)) * simulation_settings.drag_coefficient * simulation_settings.drag_reference_area;
    current_control_acc(1) -= 0.5 / holohover_props.mass * rho * state(3) * abs(state(3)) * simulation_settings.drag_coefficient * simulation_settings.drag_reference_area;

    // add force introduced by table tilt
    double g = 9.81;
    current_control_acc(0) += sin(simulation_settings.table_tilt_x * M_PI / 180.0) * g;
    current_control_acc(1) += sin(simulation_settings.table_tilt_y * M_PI / 180.0) * g;
}

void HolohoverSimulationNode::simulate_forward_callback()
{
    state = holohover.Ad * state + holohover.Bd * current_control_acc;

    holohover_msgs::msg::HolohoverStateStamped state_msg;
    state_msg.header.frame_id = "world";
    state_msg.header.stamp = this->now();
    state_msg.x = state(0);
    state_msg.y = state(1);
    state_msg.v_x = state(2);
    state_msg.v_y = state(3);
    state_msg.yaw = state(4);
    state_msg.w_z = state(5);

    if (state_msg.yaw < -M_PI)
    {
        state_msg.yaw += 2 * M_PI;
    }
    if (state_msg.yaw > M_PI)
    {
        state_msg.yaw -= 2 * M_PI;
    }

    state_publisher->publish(state_msg);
}

void HolohoverSimulationNode::imu_callback()
{
    holohover_msgs::msg::HolohoverIMUStamped imu_measurement;
    imu_measurement.header.frame_id = "body";
    imu_measurement.header.stamp = this->now();
    imu_measurement.atti.roll = 0;
    imu_measurement.atti.pitch = 0;
    imu_measurement.atti.yaw = state(4);
    imu_measurement.acc.x = current_control_acc(0);
    imu_measurement.acc.x = current_control_acc(1);
    imu_measurement.acc.z = -9.81;
    imu_measurement.gyro.x = 0;
    imu_measurement.gyro.y = 0;
    imu_measurement.gyro.z = state(5);

    std::normal_distribution<> acc_x_noise(simulation_settings.sensor_acc_bias_x, simulation_settings.sensor_acc_noise_x);
    std::normal_distribution<> acc_y_noise(simulation_settings.sensor_acc_bias_y, simulation_settings.sensor_acc_noise_y);
    std::normal_distribution<> gyro_z_noise(simulation_settings.sensor_gyro_bias_z, simulation_settings.sensor_gyro_noise_z);
    imu_measurement.acc.x += acc_x_noise(random_engine);
    imu_measurement.acc.y += acc_y_noise(random_engine);
    imu_measurement.gyro.z += gyro_z_noise(random_engine);

    imu_publisher->publish(imu_measurement);
}

void HolohoverSimulationNode::mouse_callback()
{
    Eigen::Matrix2d rotation_matrix;
    holohover.world_to_body_rotation_matrix(state, rotation_matrix);
    Eigen::Vector2d velocity_world = state.segment<2>(2);
    Eigen::Vector2d velocity_body = rotation_matrix * velocity_world;

    holohover_msgs::msg::HolohoverMouseStamped mouse_measurement;
    mouse_measurement.header.frame_id = "world";
    mouse_measurement.header.stamp = this->now();
    mouse_measurement.v_x = velocity_body(0);
    mouse_measurement.v_y = velocity_body(1);

    std::normal_distribution<> mouse_v_x_noise(0, simulation_settings.sensor_mouse_noise_x);
    std::normal_distribution<> mouse_v_y_noise(0, simulation_settings.sensor_mouse_noise_y);
    mouse_measurement.v_x += mouse_v_x_noise(random_engine);
    mouse_measurement.v_y += mouse_v_y_noise(random_engine);

    mouse_publisher->publish(mouse_measurement);
}

void HolohoverSimulationNode::pose_callback()
{
    geometry_msgs::msg::Pose2D pose_measurement;
    pose_measurement.x = state(0);
    pose_measurement.y = state(1);
    pose_measurement.theta = state(4);

    std::normal_distribution<> pose_x_noise(0, simulation_settings.sensor_pose_noise_x);
    std::normal_distribution<> pose_y_noise(0, simulation_settings.sensor_pose_noise_y);
    std::normal_distribution<> pose_yaw_noise(0, simulation_settings.sensor_pose_noise_yaw);
    pose_measurement.x += pose_x_noise(random_engine);
    pose_measurement.y += pose_y_noise(random_engine);
    pose_measurement.theta += pose_yaw_noise(random_engine);

    if (pose_measurement.theta < -M_PI)
    {
        pose_measurement.theta += 2 * M_PI;
    }
    if (pose_measurement.theta > M_PI)
    {
        pose_measurement.theta -= 2 * M_PI;
    }

    pose_publisher->publish(pose_measurement);
}

void HolohoverSimulationNode::control_callback(const holohover_msgs::msg::HolohoverControlStamped &control)
{
    current_control = control;
    calculate_control_acc();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverSimulationNode>());
    rclcpp::shutdown();
    return 0;
}
