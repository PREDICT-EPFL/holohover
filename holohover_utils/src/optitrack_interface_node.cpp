#include "optitrack_interface_node.hpp"

OptitrackInterfaceNode::OptitrackInterfaceNode() : Node("optitrack_interface")
{
    init_topics();
}

void OptitrackInterfaceNode::init_topics()
{
    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "optitrack/drone/pose", rclcpp::SensorDataQoS());

    raw_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "optitrack/drone/pose_raw", 10,
            std::bind(&OptitrackInterfaceNode::raw_pose_callback, this, std::placeholders::_1));
}

void OptitrackInterfaceNode::raw_pose_callback(const geometry_msgs::msg::PoseStamped &raw_pose)
{
    if (!is_initialized)
    {
        initial_pose = raw_pose;
        is_initialized = true;
    }

    geometry_msgs::msg::PoseStamped pose = raw_pose;
    pose.pose.position.x = raw_pose.pose.position.x - initial_pose.pose.position.x;
    pose.pose.position.y = raw_pose.pose.position.y - initial_pose.pose.position.y;
    pose.pose.position.z = 0;

    pose_publisher->publish(pose);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptitrackInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
