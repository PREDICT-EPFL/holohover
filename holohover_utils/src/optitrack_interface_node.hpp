#ifndef HOLOHOVER_UTILS_OPTITRACK_INTERFACE_NODE_HPP
#define HOLOHOVER_UTILS_OPTITRACK_INTERFACE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class OptitrackInterfaceNode : public rclcpp::Node
{
public:
    OptitrackInterfaceNode();
private:
    bool is_initialized = false;
    geometry_msgs::msg::Pose2D initial_pose;

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr raw_pose_subscription;

    void init_topics();
    void raw_pose_callback(const geometry_msgs::msg::PoseStamped &raw_pose);
};

#endif //HOLOHOVER_UTILS_OPTITRACK_INTERFACE_NODE_HPP
