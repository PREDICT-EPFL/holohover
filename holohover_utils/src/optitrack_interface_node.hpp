#ifndef HOLOHOVER_UTILS_OPTITRACK_INTERFACE_NODE_HPP
#define HOLOHOVER_UTILS_OPTITRACK_INTERFACE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "holohover_common/utils/simulation_settings.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

class OptitrackInterfaceNode : public rclcpp::Node
{
public:
    OptitrackInterfaceNode();
private:
    SimulationSettings simulation_settings;

    bool is_initialized = false;
    
    geometry_msgs::msg::PoseStamped table_pose;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr table_raw_pose_subscription;

    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> hovercraft_raw_pose_subscriptions;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>    hovercraft_pose_publishers;

    void init_topics();
    void table_raw_pose_callback(const geometry_msgs::msg::PoseStamped &raw_pose);
    void hovercraft_raw_pose_callback(std::shared_ptr<geometry_msgs::msg::PoseStamped> raw_pose, long int hovercraft_id);
};

#endif //HOLOHOVER_UTILS_OPTITRACK_INTERFACE_NODE_HPP
