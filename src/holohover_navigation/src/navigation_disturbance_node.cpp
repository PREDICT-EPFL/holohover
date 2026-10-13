#include "navigation_disturbance_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

HolohoverNavigationDisturbanceNode::HolohoverNavigationDisturbanceNode() :
    Node("navigation_disturbance"),
    holohover_props(load_holohover_pros(declare_parameter<std::string>("holohover_props_file"))),
    navigation_settings(load_navigation_settings(*this))
{
    // init zero control
    current_control.header.frame_id = "body";
    current_control.header.stamp = this->now();
    current_control.motor_a_1 = 0;
    current_control.motor_a_2 = 0;
    current_control.motor_b_1 = 0;
    current_control.motor_b_2 = 0;
    current_control.motor_c_1 = 0;
    current_control.motor_c_2 = 0;

    init_topics();
    init_timer();
}

void HolohoverNavigationDisturbanceNode::init_topics()
{
    state_publisher = this->create_publisher<holohover_msgs::msg::HolohoverStateStamped>("state", 1);
    state_disturbance_publisher = this->create_publisher<holohover_msgs::msg::HolohoverStateDisturbanceStamped>("state_disturbance", 1);

    /*imu_subscription = this->create_subscription<holohover_msgs::msg::HolohoverIMUStamped>(
            "imu",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverNavigationDisturbanceNode::imu_callback, this, std::placeholders::_1));
    mouse_subscription = this->create_subscription<holohover_msgs::msg::HolohoverMouseStamped>(
            "mouse",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverNavigationDisturbanceNode::mouse_callback, this, std::placeholders::_1));*/
    pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverNavigationDisturbanceNode::pose_callback, this, std::placeholders::_1));
    control_subscription = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(
            "control",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverNavigationDisturbanceNode::control_callback, this, std::placeholders::_1));
}

void HolohoverNavigationDisturbanceNode::init_timer()
{
    watchdog_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HolohoverNavigationDisturbanceNode::watchdog_callback, this));

    timer = this->create_wall_timer(
            std::chrono::duration<double>(navigation_settings.period),
            std::bind(&HolohoverNavigationDisturbanceNode::kalman_predict_step, this));
}

void HolohoverNavigationDisturbanceNode::init_filter(const holohover::DisturbanceHolohoverEKF::State& state)
{
    if (filter_init) return;

    holohover::DisturbanceHolohoverEKF::StateCovariance Q = holohover::DisturbanceHolohoverEKF::StateCovariance::Identity();
    holohover::DisturbanceHolohoverEKF::MeasurementCovariance R = holohover::DisturbanceHolohoverEKF::MeasurementCovariance::Identity();

    Q.diagonal() << navigation_settings.state_cov_x,
                    navigation_settings.state_cov_y,
                    navigation_settings.state_cov_yaw,
                    navigation_settings.state_cov_x_dot,
                    navigation_settings.state_cov_y_dot,
                    navigation_settings.state_cov_yaw_dot,
                    navigation_settings.state_cov_dist_x,
                    navigation_settings.state_cov_dist_y,
                    navigation_settings.state_cov_dist_yaw;
    R.diagonal() << navigation_settings.sensor_pose_cov_x,
                    navigation_settings.sensor_pose_cov_y,
                    navigation_settings.sensor_pose_cov_yaw;
    ekf = std::make_unique<holohover::DisturbanceHolohoverEKF>(Holohover(holohover_props), Q, R, state);
    last_update = this->now();
    filter_init = true;
}

void HolohoverNavigationDisturbanceNode::watchdog_callback()
{
    if (filter_init && received_control && this->now() > last_control_msg_time + std::chrono::seconds(1)) {
        ekf->update_signal(holohover::DisturbanceHolohoverEKF::Input::Zero());
    }
}

void HolohoverNavigationDisturbanceNode::kalman_predict_step()
{
    // don't publish if we have not received updates in the last 20ms
    rclcpp::Time current_time = this->now();
    if (!filter_init || (current_time - last_update).seconds() > navigation_settings.publish_time_threshold) return;

    const holohover::DisturbanceHolohoverEKF::State &state = ekf->predict_state((current_time - last_update).seconds());

    holohover_msgs::msg::HolohoverStateStamped state_msg;
    state_msg.header.frame_id = "world";
    state_msg.header.stamp = current_time;
    state_msg.state_msg.x = state(0);
    state_msg.state_msg.y = state(1);
    state_msg.state_msg.yaw = state(2);
    state_msg.state_msg.v_x = state(3);
    state_msg.state_msg.v_y = state(4);
    state_msg.state_msg.w_z = state(5);

    holohover_msgs::msg::HolohoverStateDisturbanceStamped state_disturbance_msg;
    state_disturbance_msg.header.frame_id = "world";
    state_disturbance_msg.header.stamp = current_time;
    state_disturbance_msg.state_msg.x = state(0);
    state_disturbance_msg.state_msg.y = state(1);
    state_disturbance_msg.state_msg.yaw = state(2);
    state_disturbance_msg.state_msg.v_x = state(3);
    state_disturbance_msg.state_msg.v_y = state(4);
    state_disturbance_msg.state_msg.w_z = state(5);
    state_disturbance_msg.state_msg.dist_x = state(6);
    state_disturbance_msg.state_msg.dist_y = state(7);
    state_disturbance_msg.state_msg.dist_yaw = state(8);

    state_publisher->publish(state_msg);
    state_disturbance_publisher->publish(state_disturbance_msg);
}

void HolohoverNavigationDisturbanceNode::imu_callback(const holohover_msgs::msg::HolohoverIMUStamped &measurement)
{
    current_imu = measurement;
    received_imu = true;
}

void HolohoverNavigationDisturbanceNode::mouse_callback(const holohover_msgs::msg::HolohoverMouseStamped &measurement)
{
    std::ignore = measurement;
}

void HolohoverNavigationDisturbanceNode::pose_callback(const geometry_msgs::msg::PoseStamped &measurement)
{
    tf2::Quaternion q(measurement.pose.orientation.x,
                      measurement.pose.orientation.y,
                      measurement.pose.orientation.z,
                      measurement.pose.orientation.w);

    if (!filter_init)
    {
        holohover::DisturbanceHolohoverEKF::State state;
        state.setZero();
        state(0) = measurement.pose.position.x;
        state(1) = measurement.pose.position.y;
        state(2) = tf2::getYaw(q);
        init_filter(state);
    }
    else
    {
        rclcpp::Time current_time = this->now();
        rclcpp::Time pose_time = measurement.header.stamp;
        // don't predict if update was too long ago otherwise we diverge
        if (pose_time >= last_update && (current_time - last_update).seconds() < navigation_settings.pose_update_time_threshold)
        {
            ekf->predict((pose_time - last_update).seconds());
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "EKF - NotPredicting");
            if (pose_time < last_update) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Received Message from the past: " << (pose_time - last_update).seconds() * 1e3 << " ms");
            } 
            if((current_time - last_update).seconds() >= 25e-3) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Not predicting because last update was too long ago " << (current_time - last_update).seconds() * 1e3 << " ms");
            }
        }

        holohover::DisturbanceHolohoverEKF::Measurement pose_measurement;
        pose_measurement << measurement.pose.position.x, measurement.pose.position.y, tf2::getYaw(q);
        ekf->update_measurement(pose_measurement);
        last_update = pose_time;
    }
}

void HolohoverNavigationDisturbanceNode::control_callback(const holohover_msgs::msg::HolohoverControlStamped &control)
{
    if (!filter_init) return;

    current_control = control;
    received_control = true;
    last_control_msg_time = this->now();

    holohover::DisturbanceHolohoverEKF::Input signal;
    signal << current_control.motor_a_1, current_control.motor_a_2, current_control.motor_b_1,
              current_control.motor_b_2, current_control.motor_c_1, current_control.motor_c_2;
    ekf->update_signal(signal);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverNavigationDisturbanceNode>());
    rclcpp::shutdown();
    return 0;
}
