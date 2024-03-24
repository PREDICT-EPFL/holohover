#include "navigation_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

HolohoverNavigationNode::HolohoverNavigationNode() :
    Node("navigation"),
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

    holohover::RigidBody2DEKF::StateCovariance Q = holohover::RigidBody2DEKF::StateCovariance::Identity();
    holohover::RigidBody2DEKF::MeasurementCovariance R = holohover::RigidBody2DEKF::MeasurementCovariance::Identity();

    Q.diagonal() << navigation_settings.state_cov_x,
                    navigation_settings.state_cov_y,
                    navigation_settings.state_cov_yaw,
                    navigation_settings.state_cov_x_dot,
                    navigation_settings.state_cov_y_dot,
                    navigation_settings.state_cov_yaw_dot,
                    navigation_settings.state_cov_x_dot_dot,
                    navigation_settings.state_cov_y_dot_dot,
                    navigation_settings.state_cov_yaw_dot_dot;
    R.diagonal() << navigation_settings.sensor_pose_cov_x,
                    navigation_settings.sensor_pose_cov_y,
                    navigation_settings.sensor_pose_cov_yaw;
    ekf = std::make_unique<holohover::RigidBody2DEKF>(Q, R);
    last_update = this->now();

    init_topics();
    init_timer();
}

void HolohoverNavigationNode::init_topics()
{
    state_publisher = this->create_publisher<holohover_msgs::msg::HolohoverStateStamped>("state", 1);
    state_acc_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("state_acc", 1);

    /*imu_subscription = this->create_subscription<holohover_msgs::msg::HolohoverIMUStamped>(
            "imu",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverNavigationNode::imu_callback, this, std::placeholders::_1));
    mouse_subscription = this->create_subscription<holohover_msgs::msg::HolohoverMouseStamped>(
            "mouse",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverNavigationNode::mouse_callback, this, std::placeholders::_1));*/
    pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose",
            rclcpp::SensorDataQoS(),
            std::bind(&HolohoverNavigationNode::pose_callback, this, std::placeholders::_1));
    control_subscription = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(
            "control",
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
    // don't publish if we have not received updates in the last 20ms
    rclcpp::Time current_time = this->now();
    if ((current_time - last_update).seconds() > 20e-3) return;

    const holohover::RigidBody2DEKF::State &state = ekf->predict_state((current_time - last_update).seconds());

    holohover_msgs::msg::HolohoverStateStamped state_msg;
    state_msg.header.frame_id = "world";
    state_msg.header.stamp = current_time;
    state_msg.state_msg.x = state(0);
    state_msg.state_msg.y = state(1);
    state_msg.state_msg.yaw = state(2);
    state_msg.state_msg.v_x = state(3);
    state_msg.state_msg.v_y = state(4);
    state_msg.state_msg.w_z = state(5);

    geometry_msgs::msg::Vector3Stamped state_acc_msg;
    state_acc_msg.header.frame_id = "world";
    state_acc_msg.header.stamp = current_time;
    state_acc_msg.vector.x = state(6);
    state_acc_msg.vector.y = state(7);
    state_acc_msg.vector.z = state(8);

    state_publisher->publish(state_msg);
    state_acc_publisher->publish(state_acc_msg);
}

void HolohoverNavigationNode::imu_callback(const holohover_msgs::msg::HolohoverIMUStamped &measurement)
{
    current_imu = measurement;
    received_imu = true;
}

void HolohoverNavigationNode::mouse_callback(const holohover_msgs::msg::HolohoverMouseStamped &measurement)
{
    std::ignore = measurement;
}

void HolohoverNavigationNode::pose_callback(const geometry_msgs::msg::PoseStamped &measurement)
{
    rclcpp::Time current_time = this->now();
    rclcpp::Time pose_time = measurement.header.stamp;
    // don't predict if update was too long ago otherwise we diverge
    if (pose_time > last_update && (current_time - last_update).seconds() < 25e-3)
    {
        ekf->predict((pose_time - last_update).seconds());
    }
    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "EKF - NotPredicting");
        if (pose_time <= last_update) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Received Message from the past: " << (pose_time - last_update).seconds() * 1e3 << " ms");
        } 
        if((current_time - last_update).seconds() >= 25e-3) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Not predicting because last update was too long ago " << (pose_time - last_update).seconds() * 1e3 << " ms");
        }
    }


    tf2::Quaternion q(measurement.pose.orientation.x,
                      measurement.pose.orientation.y,
                      measurement.pose.orientation.z,
                      measurement.pose.orientation.w);

    holohover::RigidBody2DEKF::Measurement pose_measurement;
    pose_measurement << measurement.pose.position.x, measurement.pose.position.y, tf2::getYaw(q);
    ekf->update(pose_measurement);
    last_update = pose_time;
}

void HolohoverNavigationNode::control_callback(const holohover_msgs::msg::HolohoverControlStamped &control)
{
    current_control = control;
    received_control = true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolohoverNavigationNode>());
    rclcpp::shutdown();
    return 0;
}
