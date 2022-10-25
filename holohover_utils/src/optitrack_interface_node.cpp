#include "optitrack_interface_node.hpp"

OptitrackInterfaceNode::OptitrackInterfaceNode() : Node("optitrack_interface")
{
    init_topics();
}

void OptitrackInterfaceNode::init_topics()
{
    pose_publisher = this->create_publisher<geometry_msgs::msg::Pose2D>(
            "optitrack/drone/pose", rclcpp::SensorDataQoS());

    raw_pose_subscription = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "optitrack/drone/pose_raw", 10,
            std::bind(&OptitrackInterfaceNode::raw_pose_callback, this, std::placeholders::_1));
}

void OptitrackInterfaceNode::raw_pose_callback(const geometry_msgs::msg::Pose2D &raw_pose)
{
    if (!is_initialized)
    {
        initial_pose = raw_pose;
        is_initialized = true;
    }

    geometry_msgs::msg::Pose2D pose;
    pose.x = raw_pose.x - initial_pose.x;
    pose.y = raw_pose.y - initial_pose.y;
    pose.theta = raw_pose.theta - initial_pose.theta;
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
