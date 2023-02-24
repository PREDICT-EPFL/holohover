#ifndef HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP
#define HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_gnc/models/holohover_model.hpp"
#include "holohover_gnc/utils/load_holohover_props.hpp"

class RvizInterfaceNode : public rclcpp::Node
{
public:
    RvizInterfaceNode();
private:
    HolohoverProps holohover_props;
    Holohover holohover;

    Holohover::state_t<double> current_state;
    Holohover::control_force_t<double> current_control;

    int marker_id_counter = 0;
    visualization_msgs::msg::Marker holohover_marker;
    visualization_msgs::msg::Marker thrust_vector_markers[6];

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_publisher;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverStateStamped>::SharedPtr state_subscription;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_subscription;

    void init_topics();
    void init_timer();
    visualization_msgs::msg::Marker create_marker(const std::string &ns, float r, float g, float b, float a = 1.0f, const std::string &frame_id = "world");
    void publish_visualization();
    void state_callback(const holohover_msgs::msg::HolohoverStateStamped &state);
    void control_callback(const holohover_msgs::msg::HolohoverControlStamped &control);
};

#endif //HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP
