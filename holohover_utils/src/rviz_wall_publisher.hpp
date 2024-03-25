#ifndef HOLOHOVER_UTILS_WALL_PUBLISHER_NODE_HPP
#define HOLOHOVER_UTILS_WALL_PUBLISHER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class RvizWallPublisher : public rclcpp::Node
{
public:
    RvizWallPublisher();
private:
    std::vector<double> table_size;

    rclcpp::TimerBase::SharedPtr timer;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_publisher;

    visualization_msgs::msg::MarkerArray wall_markers;

    void publish();
};

#endif