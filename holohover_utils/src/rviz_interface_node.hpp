#ifndef HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP
#define HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "holohover_common/utils/simulation_settings.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_msgs/msg/bool.hpp>


struct HolohoverMarkers
{
    visualization_msgs::msg::Marker holohover;
    std::array<visualization_msgs::msg::Marker, 6> thrust_vector;
    visualization_msgs::msg::Marker past_trajectory;
};

class RvizInterfaceNode : public rclcpp::Node
{
public:
    RvizInterfaceNode();
private:
    SimulationSettings simulation_settings;
    
    int marker_id_counter = 0;


    HolohoverProps holohover_props;
    Holohover holohover;

    Eigen::Matrix<double, 2, 6> motor_pos;
    Eigen::Matrix<double, 2, 6> motor_dir;

    std::vector<double> colors;

    std::vector<Holohover::state_t<double>> current_states;
    std::vector<Holohover::control_force_t<double>> current_controls;

    std::vector<HolohoverMarkers> holohover_markers;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_publisher;
    std::vector<rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr> control_subscriptions;
    std::vector<rclcpp::Subscription<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr> state_subscriptions;


    void init_topics();
    void init_timer();
    visualization_msgs::msg::Marker create_marker(const std::string &ns, float r, float g, float b, float a = 1.0f, const std::string &frame_id = "world");
    void publish_visualization();
    void control_callback(holohover_msgs::msg::HolohoverControlStamped::SharedPtr msg, long int hovercraft_id);
    void state_callback(holohover_msgs::msg::HolohoverStateStamped::SharedPtr state_msg, long int hovercraft_id);
    HolohoverMarkers init_holohover_markers(std::string name, std::vector<double> colors);
    void init_props();

    // Update Sep 30: puck simulation
    Holohover::state_t<double> current_puck_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr puck_pose_subscription;
    void puck_pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr state_msg);
    visualization_msgs::msg::Marker puck_marker;
    void init_puck_marker();

    // Update Sep 30: collision detector
    bool current_wall_collision_state;  // Variables to store the current collision states
    bool current_hovercraft_collision_state;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wall_collided_subscription;    // Subscriptions for collision detection
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hovercraft_collided_subscription;
    void wall_collided_callback(std_msgs::msg::Bool::SharedPtr msg);    // Callback functions for collision events
    void hovercraft_collided_callback(std_msgs::msg::Bool::SharedPtr msg);

};

#endif //HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP
