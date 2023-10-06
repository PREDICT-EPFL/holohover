#ifndef HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP
#define HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_msgs/msg/holohover_trajectory.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "holohover_common/utils/load_holohover_props.hpp"

class RvizInterfaceNode : public rclcpp::Node
{
public:
    RvizInterfaceNode();
private:
    HolohoverProps holohover_props;
    Holohover holohover;

    Holohover::state_t<double> current_state;
    Holohover::state_t<double> next_state[20];
    Holohover::state_t<double> current_ref;
    //geometry_msgs::msg::Pose2D ref;
    holohover_msgs::msg::HolohoverState ref;
    Holohover::control_force_t<double> current_control;

    int marker_id_counter = 0;
    visualization_msgs::msg::Marker holohover_marker;
    visualization_msgs::msg::Marker reference_holohover_marker;
    visualization_msgs::msg::Marker past_trajectory_marker;
    visualization_msgs::msg::Marker next_pos_marker[20];
    visualization_msgs::msg::Marker next_vel_marker[20];
    visualization_msgs::msg::Marker reference_direction_marker;
    visualization_msgs::msg::Marker reference_velocity_marker;
    visualization_msgs::msg::Marker reference_marker;
    visualization_msgs::msg::Marker thrust_vector_markers[6];

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_subscription;    
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverTrajectory>::SharedPtr trajectory_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverState>::SharedPtr reference_subscription;


    void init_topics();
    void init_timer();
    visualization_msgs::msg::Marker create_marker(const std::string &ns, float r, float g, float b, float a = 1.0f, const std::string &frame_id = "world");
    void publish_visualization();
    void state_callback(const holohover_msgs::msg::HolohoverStateStamped &state);
    void trajectory_callback(const holohover_msgs::msg::HolohoverTrajectory &state_trajectory);
    void ref_callback(const holohover_msgs::msg::HolohoverState &pose);
    void control_callback(const holohover_msgs::msg::HolohoverControlStamped &control);
};

#endif //HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP
