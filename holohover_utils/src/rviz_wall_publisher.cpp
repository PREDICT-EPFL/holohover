#include "rviz_wall_publisher.hpp"


RvizWallPublisher::RvizWallPublisher() :
    Node("rviz_wall_publisher"),
    table_size(declare_parameter<std::vector<double>>("table_size"))
{
    viz_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/drone", 10);

    // Publish marker for wall visualization
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();

    marker.header.frame_id = "world";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "walls";
    marker.id = 1234;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.25;
    marker.color.a = 1.0;
    
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = .01;

    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

    geometry_msgs::msg::Point p1;
    p1.z = 0.f;


    p1.x = 0.0f;
    p1.y = 0.0f;
    marker.points.push_back(p1);

    p1.x = table_size[0];
    p1.y = 0;
    marker.points.push_back(p1);

    p1.x = table_size[0];
    p1.y = table_size[1];
    marker.points.push_back(p1);

    p1.x = 0;
    p1.y = table_size[1];
    marker.points.push_back(p1);

    p1.x = 0;
    p1.y = 0;
    marker.points.push_back(p1);

    wall_markers.markers.push_back(marker);


    timer = this->create_wall_timer(
            std::chrono::duration<double>(1),
            std::bind(&RvizWallPublisher::publish, this));
}


void RvizWallPublisher::publish()
{
    viz_publisher->publish(wall_markers);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizWallPublisher>());
    rclcpp::shutdown();
    return 0;
}
