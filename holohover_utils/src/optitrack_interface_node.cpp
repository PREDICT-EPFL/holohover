#include "optitrack_interface_node.hpp"

OptitrackInterfaceNode::OptitrackInterfaceNode() : Node("optitrack_interface")
{
    init_topics();
}

void OptitrackInterfaceNode::init_topics()
{
    pose_publisher = this->create_publisher<geometry_msgs::msg::Pose2D>(
            "optitrack/drone/pose", rclcpp::SensorDataQoS());

    raw_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vrpn_mocap/Holohover/pose", 10,
            std::bind(&OptitrackInterfaceNode::raw_pose_callback, this, std::placeholders::_1));
}

void OptitrackInterfaceNode::raw_pose_callback(const geometry_msgs::msg::PoseStamped &raw_pose)
{
    tf2::Quaternion orientation;
    tf2::fromMsg(raw_pose.pose.orientation, orientation);
    tf2::Matrix3x3 rotation_matrix(orientation);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);

    if (!is_initialized)
    {
        initial_pose.x = -raw_pose.pose.position.y;
        initial_pose.y = raw_pose.pose.position.x;
        initial_pose.theta = yaw;
        is_initialized = true;
    }

    geometry_msgs::msg::Pose2D pose;
    pose.x = -raw_pose.pose.position.y - initial_pose.x;
    pose.y = raw_pose.pose.position.x - initial_pose.y;
    pose.theta = yaw - initial_pose.theta;
    if (pose.theta < -M_PI)
    {
        pose.theta += 2 * M_PI;
    }
    if (pose.theta > M_PI)
    {
        pose.theta -= 2 * M_PI;
    }

    pose_publisher->publish(pose);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptitrackInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
