#ifndef HOLOHOVER_UTILS_OPTITRACK_INTERFACE_NODE_HPP
#define HOLOHOVER_UTILS_OPTITRACK_INTERFACE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "holohover_common/utils/simulation_settings.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <cmath>

class OptitrackInterfaceNode : public rclcpp::Node
{
public:
    OptitrackInterfaceNode();
private:
    SimulationSettings simulation_settings;

    bool is_initialized = false;
    
    geometry_msgs::msg::PoseStamped table_pose;
    std::vector<double> table_center;  // the table frame's origin in the table marker's frame

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr table_raw_pose_subscription;

    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> hovercraft_raw_pose_subscriptions;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>    hovercraft_pose_publishers;

    // Debug: a raw pose in the table frame, height and tilt kept (/optitrack/<name>_pose_raw_shifted), and its
    // roll, pitch, yaw [rad] (/optitrack/<name>_rpy_shifted), to compare with the EKF and see a mis-solved rigid body
    struct ShiftedPublishers {
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy;
    };
    ShiftedPublishers puck_shifted_publishers;
    std::vector<ShiftedPublishers> hovercraft_shifted_publishers;
    ShiftedPublishers create_shifted_publishers(const std::string &name);
    void publish_shifted(const geometry_msgs::msg::PoseStamped &raw_pose, const ShiftedPublishers &publishers) const;

    void init_topics();
    geometry_msgs::msg::PoseStamped to_table_frame(const geometry_msgs::msg::PoseStamped &raw_pose) const;
    void table_raw_pose_callback(const geometry_msgs::msg::PoseStamped &raw_pose);
    void hovercraft_raw_pose_callback(std::shared_ptr<geometry_msgs::msg::PoseStamped> raw_pose, long int hovercraft_id);

    // Update Sep 30: puck simulation
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr puck_raw_pose_subscription;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr puck_pose_publisher;
    void puck_raw_pose_callback(const geometry_msgs::msg::PoseStamped &raw_pose);
};

#endif //HOLOHOVER_UTILS_OPTITRACK_INTERFACE_NODE_HPP
