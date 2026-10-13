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

        hovercraft_raw_pose_subscriptions.push_back(sub);

        // pose publishers
        topic_name = "/" + simulation_settings.hovercraft_names[i] + "/pose";
        hovercraft_pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10));
    }

    table_raw_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/optitrack/table_pose_raw", rclcpp::SensorDataQoS(),
        std::bind(&OptitrackInterfaceNode::table_raw_pose_callback, this, std::placeholders::_1));

    // Update Sep 30: puck simulation
    puck_raw_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/optitrack/puck_pose_raw", rclcpp::SensorDataQoS(),
        std::bind(&OptitrackInterfaceNode::puck_raw_pose_callback, this, std::placeholders::_1));
    puck_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/puck/pose", 10);
}

// Update Sep 30: puck simulation
void OptitrackInterfaceNode::puck_raw_pose_callback(const geometry_msgs::msg::PoseStamped &raw_pose)
{
    geometry_msgs::msg::PoseStamped pose;

    pose.header = raw_pose.header;

    pose.pose.position.x = raw_pose.pose.position.x - table_pose.pose.position.x;
    pose.pose.position.y = raw_pose.pose.position.y - table_pose.pose.position.y;
    pose.pose.position.z = 0;
    
    tf2::Quaternion q_raw, q_table, q_new;

    tf2::convert(raw_pose.pose.orientation, q_raw);
    tf2::convert(table_pose.pose.orientation, q_table);

    double r,p,y_raw,y_table;
    tf2::Matrix3x3 m_raw(q_raw);
    m_raw.getRPY(r,p,y_raw);

    tf2::Matrix3x3 m_table(q_table);
    m_table.getRPY(r,p,y_table);

    q_new.setRPY(0, 0, y_raw - y_table);
    q_new.normalize();

    tf2::convert(q_new, pose.pose.orientation);

    puck_pose_publisher->publish(pose);
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
    
    tf2::Quaternion q_raw, q_table, q_new;

    tf2::convert(raw_pose->pose.orientation, q_raw);
    tf2::convert(table_pose.pose.orientation, q_table);

    double r,p,y_raw,y_table;
    tf2::Matrix3x3 m_raw(q_raw);
    m_raw.getRPY(r,p,y_raw);

    tf2::Matrix3x3 m_table(q_table);
    m_table.getRPY(r,p,y_table);

    q_new.setRPY(0, 0, y_raw - y_table);
    q_new.normalize();

    tf2::convert(q_new, pose.pose.orientation);

    hovercraft_pose_publishers[hovercraft_id]->publish(pose);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptitrackInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
