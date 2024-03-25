#include "optitrack_interface_node.hpp"

OptitrackInterfaceNode::OptitrackInterfaceNode() : 
    Node("optitrack_interface"),
    simulation_settings(load_simulation_settings(*this))
{
    init_topics();
}

void OptitrackInterfaceNode::init_topics()
{
    for(size_t i = 0; i < simulation_settings.hovercraft_ids.size(); i++) {
        RCLCPP_INFO(get_logger(), "Hovercraft id: %ld", simulation_settings.hovercraft_ids[i]);

        // pose_raw subscriptions
        auto topic_name = "/optitrack/" + simulation_settings.hovercraft_names[i] + "_pose_raw";

        std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> callback = 
                 std::bind(&OptitrackInterfaceNode::hovercraft_raw_pose_callback, this, std::placeholders::_1, i);

        auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(topic_name, rclcpp::SensorDataQoS(), callback);

        hovercrafts_raw_pose_subscriptions.push_back(sub);

        // pose publishers
        topic_name = "/" + simulation_settings.hovercraft_names[i] + "/pose";
        hovercrafts_pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic_name, rclcpp::SensorDataQoS()));
    }

    table_raw_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/optitrack/table_pose_raw", 10,
        std::bind(&OptitrackInterfaceNode::table_raw_pose_callback, this, std::placeholders::_1));
}

void OptitrackInterfaceNode::table_raw_pose_callback(const geometry_msgs::msg::PoseStamped &raw_pose)
{
    if (!is_initialized)
        is_initialized = true;
    
    table_pose = raw_pose;
}

void OptitrackInterfaceNode::hovercraft_raw_pose_callback(std::shared_ptr<geometry_msgs::msg::PoseStamped> raw_pose, long int hovercraft_id)
{
    if (!is_initialized)
        return;

    geometry_msgs::msg::PoseStamped pose;

    pose.header = raw_pose->header;

    pose.pose.position.x = raw_pose->pose.position.x - table_pose.pose.position.x;
    pose.pose.position.y = raw_pose->pose.position.y - table_pose.pose.position.y;
    pose.pose.position.z = 0;
    

    double r, p, y_hov, y_table;    
    tf2::Quaternion q;

    tf2::fromMsg(table_pose.pose.orientation, q);
    tf2::Matrix3x3 m_table(q);
    m_table.getRPY(r, p, y_table);

    tf2::fromMsg(table_pose.pose.orientation, q);
    tf2::Matrix3x3 m_hov(q);
    m_table.getRPY(r, p, y_hov);

    q.setRPY(0, 0, y_hov - y_table);
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    
    hovercrafts_pose_publishers[hovercraft_id]->publish(pose);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptitrackInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
