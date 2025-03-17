#include "rviz_interface_node.hpp"


RvizInterfaceNode::RvizInterfaceNode() :
        Node("rviz_interface"),
        simulation_settings(load_simulation_settings(*this)),
        holohover_props(load_holohover_pros(declare_parameter<std::string>("rviz_props_file"))),
        holohover(holohover_props),
        colors(declare_parameter<std::vector<double>>("color"))
{
    RCLCPP_INFO(get_logger(), "Starting rviz interface node.");

    init_props();
    init_topics();
    init_wall_markers();
    init_timer();
}


void RvizInterfaceNode::publish_visualization()
{
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.reserve(8 * simulation_settings.hovercraft_ids.size());

    auto header_now = now();

    for(size_t i = 0; i < simulation_settings.hovercraft_ids.size(); i++) {
        //RCLCPP_INFO(get_logger(), "Publish visualization - Hovercraft id: %ld", simulation_settings.hovercraft_ids[i]);
        
        // body to world rotation matrix
        Eigen::Matrix2d rotation_matrix;
        holohover.body_to_world_rotation_matrix(current_states[i], rotation_matrix);

        // Holohover model
        holohover_markers[i].holohover.header.stamp = header_now;
        holohover_markers[i].holohover.pose.position.x = current_states[i](0);
        holohover_markers[i].holohover.pose.position.y = current_states[i](1);
        holohover_markers[i].holohover.pose.position.z = 0.015; // center holohover 3d mdoel
        tf2::Quaternion q;
        q.setRPY(0, 0, current_states[i](4));
        holohover_markers[i].holohover.pose.orientation.x = q.getX();
        holohover_markers[i].holohover.pose.orientation.y = q.getY();
        holohover_markers[i].holohover.pose.orientation.z = q.getZ();
        holohover_markers[i].holohover.pose.orientation.w = q.getW();

        // past_trajectory
        holohover_markers[i].past_trajectory.header.stamp = header_now;
        holohover_markers[i].past_trajectory.id = marker_id_counter++;
        holohover_markers[i].past_trajectory.pose.position.x = current_states[i](0);
        holohover_markers[i].past_trajectory.pose.position.y = current_states[i](1);

        // thrust arrows
        double propeller_height = 0.045;
        double thrust_scaling = 0.2;
        double min_display_force = 0.01; 

        for (int j = 0; j < 3; j++) {
            Eigen::Vector2d start_point_1 = rotation_matrix * motor_pos.col(2 * j);
            Eigen::Vector2d start_point_2 = rotation_matrix * motor_pos.col(2 * j + 1);

            Eigen::Vector2d end_point_1 = rotation_matrix * (motor_pos.col(2 * j) - motor_dir.col(2 * j) * fmax(current_controls[i](2 * j), min_display_force) * thrust_scaling);
            Eigen::Vector2d end_point_2 = rotation_matrix * (motor_pos.col(2 * j + 1) - motor_dir.col(2 * j + 1) * fmax(current_controls[i](2 * j + 1), min_display_force) * thrust_scaling);

            holohover_markers[i].thrust_vector[2 * j].header.stamp = header_now;
            holohover_markers[i].thrust_vector[2 * j].points[0].x = start_point_1(0) + current_states[i](0);
            holohover_markers[i].thrust_vector[2 * j].points[0].y = start_point_1(1) + current_states[i](1);
            holohover_markers[i].thrust_vector[2 * j].points[0].z = propeller_height;
            holohover_markers[i].thrust_vector[2 * j].points[1].x = end_point_1(0) + current_states[i](0);
            holohover_markers[i].thrust_vector[2 * j].points[1].y = end_point_1(1) + current_states[i](1);
            holohover_markers[i].thrust_vector[2 * j].points[1].z = propeller_height;

            holohover_markers[i].thrust_vector[2 * j + 1].header.stamp = header_now;
            holohover_markers[i].thrust_vector[2 * j + 1].points[0].x = start_point_2(0) + current_states[i](0);
            holohover_markers[i].thrust_vector[2 * j + 1].points[0].y = start_point_2(1) + current_states[i](1);
            holohover_markers[i].thrust_vector[2 * j + 1].points[0].z = propeller_height;
            holohover_markers[i].thrust_vector[2 * j + 1].points[1].x = end_point_2(0) + current_states[i](0);
            holohover_markers[i].thrust_vector[2 * j + 1].points[1].y = end_point_2(1) + current_states[i](1);
            holohover_markers[i].thrust_vector[2 * j + 1].points[1].z = propeller_height;
        }

        markers.markers.push_back(holohover_markers[i].holohover);

        markers.markers.push_back(holohover_markers[i].past_trajectory);

        for (auto &thrust_vector_marker : holohover_markers[i].thrust_vector)
        {
            markers.markers.push_back(thrust_vector_marker);
        }
    }
    viz_publisher->publish(markers);
}

void RvizInterfaceNode::init_topics()
{
    viz_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/visualization/drone", 10);

    for(size_t i = 0; i < simulation_settings.hovercraft_ids.size(); i++) {
        RCLCPP_INFO(get_logger(), "Init topics - Hovercraft id: %ld", simulation_settings.hovercraft_ids[i]);

        // control subscriptions
        auto topic_name = "/" + simulation_settings.hovercraft_names[i] + "/control";

        std::function<void(const holohover_msgs::msg::HolohoverControlStamped::SharedPtr)> callback = 
                std::bind(&RvizInterfaceNode::control_callback, this, std::placeholders::_1, i);

        control_subscriptions.push_back(
            this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(topic_name, rclcpp::SensorDataQoS(), callback));

        // STATE subscriptions
        topic_name = "/" + simulation_settings.hovercraft_names[i] + "/state";

        std::function<void(const holohover_msgs::msg::HolohoverStateStamped::SharedPtr)> callback2 = 
                std::bind(&RvizInterfaceNode::state_callback, this, std::placeholders::_1, i);

        state_subscriptions.push_back(
            this->create_subscription<holohover_msgs::msg::HolohoverStateStamped>(topic_name, rclcpp::SensorDataQoS(), callback2));
        

        // MARKERS
        std::vector<double> col = std::vector<double>(colors.begin() + i * 4, colors.begin() + (i + 1) * 4);;
        holohover_markers.push_back(init_holohover_markers(simulation_settings.hovercraft_names[i], col)); //simulation_settings.colors[i]));

        Holohover::state_t<double> state;
        current_states.push_back(state);

        Holohover::control_force_t<double> control;
        control.setZero();
        current_controls.push_back(control);
    }
}

HolohoverMarkers RvizInterfaceNode::init_holohover_markers(std::string name, std::vector<double> colors)
{
    HolohoverMarkers holohover_markers;

    holohover_markers.holohover = create_marker(name, 0.75, 0.75, 0.75);
    holohover_markers.holohover.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    holohover_markers.holohover.mesh_resource = "package://holohover_utils/gui/holohover.stl";
    holohover_markers.holohover.scale.x = 1;
    holohover_markers.holohover.scale.y = 1;
    holohover_markers.holohover.scale.z = 1;
    holohover_markers.holohover.color.r = colors[0];
    holohover_markers.holohover.color.g = colors[1];
    holohover_markers.holohover.color.b = colors[2];
    holohover_markers.holohover.color.a = colors[3];

    for (int i = 0; i < 6; i++)
    {
        holohover_markers.thrust_vector[i] = create_marker(name, 1.0, 0.5, 0.0);
        holohover_markers.thrust_vector[i].type = visualization_msgs::msg::Marker::ARROW;
        holohover_markers.thrust_vector[i].scale.x = 0.01;
        holohover_markers.thrust_vector[i].scale.y = 0.02;
        holohover_markers.thrust_vector[i].scale.z = 0.02;
        holohover_markers.thrust_vector[i].points.resize(2);
    }

    holohover_markers.past_trajectory = create_marker(name, 0.75, 0.5, 0);
    holohover_markers.past_trajectory.type = visualization_msgs::msg::Marker::CUBE;
    holohover_markers.past_trajectory.scale.x = 0.005;
    holohover_markers.past_trajectory.scale.y = 0.005;
    holohover_markers.past_trajectory.scale.z = 0.005;
    holohover_markers.past_trajectory.lifetime = rclcpp::Duration(2,0);

    return holohover_markers;
}

void RvizInterfaceNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(0.04),
            std::bind(&RvizInterfaceNode::publish_visualization, this));

    wall_timer = this->create_wall_timer(
        std::chrono::duration<double>(1),
        std::bind(&RvizInterfaceNode::publish_walls, this));
}

visualization_msgs::msg::Marker RvizInterfaceNode::create_marker(const std::string &ns, float r, float g, float b, float a, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = ns;
    marker.id = marker_id_counter++;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    return marker;
}

void RvizInterfaceNode::state_callback(holohover_msgs::msg::HolohoverStateStamped::SharedPtr state_msg, long int hovercraft_id)
{
    // RCLCPP_INFO(get_logger(), "STATE CALLBACK - Hovercraft id: %ld", hovercraft_id);

    current_states[hovercraft_id](0) = state_msg->state_msg.x;
    current_states[hovercraft_id](1) = state_msg->state_msg.y;
    current_states[hovercraft_id](2) = state_msg->state_msg.v_x;
    current_states[hovercraft_id](3) = state_msg->state_msg.v_y;
    current_states[hovercraft_id](4) = state_msg->state_msg.yaw;
    current_states[hovercraft_id](5) = state_msg->state_msg.w_z;
}

void RvizInterfaceNode::control_callback(holohover_msgs::msg::HolohoverControlStamped::SharedPtr control, long int hovercraft_id) 
{
    // RCLCPP_INFO(get_logger(), "CONTROL CALLBACK - Hovercraft id: %ld", hovercraft_id);

    current_controls[hovercraft_id](0) = control->motor_a_1;
    current_controls[hovercraft_id](1) = control->motor_a_2;
    current_controls[hovercraft_id](2) = control->motor_b_1;
    current_controls[hovercraft_id](3) = control->motor_b_2;
    current_controls[hovercraft_id](4) = control->motor_c_1;
    current_controls[hovercraft_id](5) = control->motor_c_2;
}

void RvizInterfaceNode::init_props()
{
    RCLCPP_INFO(get_logger(), "INIT PROPS");

    // position vector for the propeller pairs
    motor_pos(0, 0) = holohover_props.motor_pos_a_1[0];
    motor_pos(1, 0) = holohover_props.motor_pos_a_1[1];
    motor_pos(0, 1) = holohover_props.motor_pos_a_2[0];
    motor_pos(1, 1) = holohover_props.motor_pos_a_2[1];
    motor_pos(0, 2) = holohover_props.motor_pos_b_1[0];
    motor_pos(1, 2) = holohover_props.motor_pos_b_1[1];
    motor_pos(0, 3) = holohover_props.motor_pos_b_2[0];
    motor_pos(1, 3) = holohover_props.motor_pos_b_2[1];
    motor_pos(0, 4) = holohover_props.motor_pos_c_1[0];
    motor_pos(1, 4) = holohover_props.motor_pos_c_1[1];
    motor_pos(0, 5) = holohover_props.motor_pos_c_2[0];
    motor_pos(1, 5) = holohover_props.motor_pos_c_2[1];

    // motor force direction vectors
    motor_dir(0, 0) = holohover_props.learned_motor_vec_a_1[0];
    motor_dir(1, 0) = holohover_props.learned_motor_vec_a_1[1];
    motor_dir(0, 1) = holohover_props.learned_motor_vec_a_2[0];
    motor_dir(1, 1) = holohover_props.learned_motor_vec_a_2[1];
    motor_dir(0, 2) = holohover_props.learned_motor_vec_b_1[0];
    motor_dir(1, 2) = holohover_props.learned_motor_vec_b_1[1];
    motor_dir(0, 3) = holohover_props.learned_motor_vec_b_2[0];
    motor_dir(1, 3) = holohover_props.learned_motor_vec_b_2[1];
    motor_dir(0, 4) = holohover_props.learned_motor_vec_c_1[0];
    motor_dir(1, 4) = holohover_props.learned_motor_vec_c_1[1];
    motor_dir(0, 5) = holohover_props.learned_motor_vec_c_2[0];
    motor_dir(1, 5) = holohover_props.learned_motor_vec_c_2[1];
}

void RvizInterfaceNode::init_wall_markers(){
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


    p1.x = -simulation_settings.table_size[0] / 2.0f;
    p1.y = -simulation_settings.table_size[1] / 2.0f;
    marker.points.push_back(p1);

    p1.x =  simulation_settings.table_size[0] / 2.0f;
    p1.y = -simulation_settings.table_size[1] / 2.0f;
    marker.points.push_back(p1);

    p1.x = simulation_settings.table_size[0] / 2.0f;
    p1.y = simulation_settings.table_size[1] / 2.0f;
    marker.points.push_back(p1);

    p1.x = -simulation_settings.table_size[0] / 2.0f;
    p1.y =  simulation_settings.table_size[1] / 2.0f;
    marker.points.push_back(p1);

    p1.x = -simulation_settings.table_size[0] / 2.0f;
    p1.y = -simulation_settings.table_size[1] / 2.0f;
    marker.points.push_back(p1);

    wall_markers.markers.push_back(marker);
}

void RvizInterfaceNode::publish_walls(){
    viz_publisher->publish(wall_markers);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}