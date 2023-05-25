#include "rviz_interface_node.hpp"

RvizInterfaceNode::RvizInterfaceNode() :
        Node("rviz_interface", rclcpp::NodeOptions().allow_undeclared_parameters(true)
                                                    .automatically_declare_parameters_from_overrides(true)),
        holohover_props(load_holohover_pros(*this)),
        holohover(holohover_props)
{
    holohover_marker = create_marker("holohover", 0.75, 0.75, 0.75);
    holohover_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    holohover_marker.mesh_resource = "package://holohover_utils/gui/holohover.stl";
    holohover_marker.scale.x = 1;
    holohover_marker.scale.y = 1;
    holohover_marker.scale.z = 1;

    for (int i = 0; i < 6; i++)
    {
        thrust_vector_markers[i] = create_marker("thrust vector " + std::to_string(i), 1.0, 0.5, 0.0);
        thrust_vector_markers[i].type = visualization_msgs::msg::Marker::ARROW;
        thrust_vector_markers[i].scale.x = 0.01;
        thrust_vector_markers[i].scale.y = 0.02;
        thrust_vector_markers[i].scale.z = 0.02;
        thrust_vector_markers[i].points.resize(2);
    }
    past_trajectory_marker = create_marker("past_trajectory", 0.75, 0.5, 0);
    past_trajectory_marker.type = visualization_msgs::msg::Marker::CUBE;
    past_trajectory_marker.scale.x = 0.005;
    past_trajectory_marker.scale.y = 0.005;
    past_trajectory_marker.scale.z = 0.005;
    past_trajectory_marker.lifetime = rclcpp::Duration(10,0);

    reference_marker = create_marker("reference", 0.75, 0, 0);
    reference_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    reference_marker.scale.x = 0.02;
    reference_marker.scale.y = 0.02;
    reference_marker.scale.z = 0.02;

    reference_direction_marker = create_marker("reference_direction", 0.75, 0.75, 0);
    reference_direction_marker.type = visualization_msgs::msg::Marker::ARROW;
    reference_direction_marker.scale.x = 0.01;
    reference_direction_marker.scale.y = 0.02;
    reference_direction_marker.scale.z = 0.02;
    reference_direction_marker.points.resize(2);

    for (int i = 0; i < 20; i++)
    {
        next_state_marker[i] = create_marker("next_state"+ std::to_string(i), 0, 0, 0.75);
        next_state_marker[i].type = visualization_msgs::msg::Marker::CYLINDER;
        next_state_marker[i].scale.x = 0.005;
        next_state_marker[i].scale.y = 0.005;
        next_state_marker[i].scale.z = 0.1;
        //next_state_marker[i].lifetime = rclcpp::Duration(2,0);
    }

    init_topics();
    init_timer();
}

void RvizInterfaceNode::init_topics()
{
    viz_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization/drone", 10);

    state_subscription = this->create_subscription<holohover_msgs::msg::HolohoverStateStamped>(
            "navigation/state", 10,
            std::bind(&RvizInterfaceNode::state_callback, this, std::placeholders::_1));
    
    trajectory_subscription = this->create_subscription<holohover_msgs::msg::HolohoverTrajectory>(
            "control/HolohoverTrajectory", rclcpp::SensorDataQoS(),
            std::bind(&RvizInterfaceNode::trajectory_callback, this, std::placeholders::_1));
    
    reference_subscription = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "control/ref", 10,
            std::bind(&RvizInterfaceNode::ref_callback, this, std::placeholders::_1));
    
    control_subscription = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(
            "drone/control", rclcpp::SensorDataQoS(),
            std::bind(&RvizInterfaceNode::control_callback, this, std::placeholders::_1));
    
}

void RvizInterfaceNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(0.04),
            std::bind(&RvizInterfaceNode::publish_visualization, this));
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

void RvizInterfaceNode::publish_visualization()
{
    // Holohover model
    holohover_marker.header.stamp = now();
    holohover_marker.pose.position.x = current_state(0)-holohover_props.CoM[0];
    holohover_marker.pose.position.y = current_state(1)-holohover_props.CoM[1];
    holohover_marker.pose.position.z = 0.015; // center holohover 3d mdoel
    tf2::Quaternion q;
    q.setRPY(0, 0, current_state(4));
    holohover_marker.pose.orientation.x = q.getX();
    holohover_marker.pose.orientation.y = q.getY();
    holohover_marker.pose.orientation.z = q.getZ();
    holohover_marker.pose.orientation.w = q.getW();

    // past_trajectory
    //past_trajectory_marker.header.stamp = now();
    past_trajectory_marker = create_marker("past_trajectory", 0.75, 0.5, 0);
    past_trajectory_marker.type = visualization_msgs::msg::Marker::CUBE;
    past_trajectory_marker.scale.x = 0.005;
    past_trajectory_marker.scale.y = 0.005;
    past_trajectory_marker.scale.z = 0.005;
    past_trajectory_marker.lifetime = rclcpp::Duration(10,0);
    past_trajectory_marker.pose.position.x = current_state(0);
    past_trajectory_marker.pose.position.y = current_state(1);

    reference_marker.pose.position.x = ref.x;
    reference_marker.pose.position.y = ref.y;

    reference_direction_marker.points[0].x = ref.x;
    reference_direction_marker.points[0].y = ref.y;
    reference_direction_marker.points[1].x = ref.x+0.05*cos(ref.theta);
    reference_direction_marker.points[1].y = ref.y+0.05*sin(ref.theta);


    // thrust arrows
    double propeller_height = 0.045;
    double thrust_scaling = 0.2;
    double min_display_force = 0.01; 
    for (int i = 0; i < 3; i++) {
        // position vector for the propeller pair i
        Eigen::Vector2d position_1;
        position_1(0) = holohover_props.propeller_pair_radial_distance * cos(holohover_props.phi_offset + holohover.phi * i - holohover_props.angle_propeller_pair);
        position_1(1) = holohover_props.propeller_pair_radial_distance * sin(holohover_props.phi_offset + holohover.phi * i - holohover_props.angle_propeller_pair);
        Eigen::Vector2d position_2;
        position_2(0) = holohover_props.propeller_pair_radial_distance * cos(holohover_props.phi_offset + holohover.phi * i + holohover_props.angle_propeller_pair);
        position_2(1) = holohover_props.propeller_pair_radial_distance * sin(holohover_props.phi_offset + holohover.phi * i + holohover_props.angle_propeller_pair);
        // inverse propeller force direction for the propeller pair i
        Eigen::Vector2d direction_1;
        direction_1(0) = sin(holohover_props.phi_offset + holohover.phi * i);
        direction_1(1) = -cos(holohover_props.phi_offset + holohover.phi * i);
        Eigen::Vector2d direction_2;
        direction_2(0) = -sin(holohover_props.phi_offset + holohover.phi * i);
        direction_2(1) = cos(holohover_props.phi_offset + holohover.phi * i);
        // body to world rotation matrix
        Eigen::Matrix2d rotation_matrix;
        holohover.body_to_world_rotation_matrix(current_state, rotation_matrix);

        Eigen::Vector2d start_point_1 = rotation_matrix * position_1;
        Eigen::Vector2d start_point_2 = rotation_matrix * position_2;

        Eigen::Vector2d end_point_1 = rotation_matrix * (position_1 + direction_1 * fmax(current_control(2 * i), min_display_force) * thrust_scaling);
        Eigen::Vector2d end_point_2 = rotation_matrix * (position_2 + direction_2 * fmax(current_control(2 * i + 1), min_display_force) * thrust_scaling);

        thrust_vector_markers[2 * i].header.stamp = now();
        thrust_vector_markers[2 * i].points[0].x = start_point_1(0) + current_state(0);
        thrust_vector_markers[2 * i].points[0].y = start_point_1(1) + current_state(1);
        thrust_vector_markers[2 * i].points[0].z = propeller_height;
        thrust_vector_markers[2 * i].points[1].x = end_point_1(0) + current_state(0);
        thrust_vector_markers[2 * i].points[1].y = end_point_1(1) + current_state(1);
        thrust_vector_markers[2 * i].points[1].z = propeller_height;

        thrust_vector_markers[2 * i + 1].header.stamp = now();
        thrust_vector_markers[2 * i + 1].points[0].x = start_point_2(0) + current_state(0);
        thrust_vector_markers[2 * i + 1].points[0].y = start_point_2(1) + current_state(1);
        thrust_vector_markers[2 * i + 1].points[0].z = propeller_height;
        thrust_vector_markers[2 * i + 1].points[1].x = end_point_2(0) + current_state(0);
        thrust_vector_markers[2 * i + 1].points[1].y = end_point_2(1) + current_state(1);
        thrust_vector_markers[2 * i + 1].points[1].z = propeller_height;
    }

    for (int i = 0; i < 20; i++) {
        // next_state_marker[i].lifetime = rclcpp::Duration(2,0);
        next_state_marker[i].pose.position.x = next_state[i](0);
        next_state_marker[i].pose.position.y = next_state[i](1);
    }

    // publish markers
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.reserve(10);
    markers.markers.push_back(holohover_marker);
    markers.markers.push_back(past_trajectory_marker);
    markers.markers.push_back(reference_marker);
    markers.markers.push_back(reference_direction_marker);
    //markers.markers.push_back(next_state_marker);

    for (auto &thrust_vector_marker : thrust_vector_markers)
    {
        markers.markers.push_back(thrust_vector_marker);
    }

    for (auto &next_state_marker : next_state_marker)
    {
        markers.markers.push_back(next_state_marker);
    }

    viz_publisher->publish(markers);
}

void RvizInterfaceNode::state_callback(const holohover_msgs::msg::HolohoverStateStamped &state_msg)
{
    current_state(0) = state_msg.state_msg.x ;
    current_state(1) = state_msg.state_msg.y ;
    current_state(2) = state_msg.state_msg.v_x;
    current_state(3) = state_msg.state_msg.v_y;
    current_state(4) = state_msg.state_msg.yaw;
    current_state(5) = state_msg.state_msg.w_z;
}

void RvizInterfaceNode::trajectory_callback(const holohover_msgs::msg::HolohoverTrajectory &state_trajectory)
{
    //next_state = state_trajectory.state_trajectory[0];
    for (int i = 0; i < 20; i++) {
        next_state[i](0) = state_trajectory.state_trajectory[i].x ;
        next_state[i](1) = state_trajectory.state_trajectory[i].y ;
        next_state[i](2) = state_trajectory.state_trajectory[i].v_x ;
        next_state[i](3) = state_trajectory.state_trajectory[i].v_y ;
        next_state[i](4) = state_trajectory.state_trajectory[i].yaw ;
        next_state[i](5) = state_trajectory.state_trajectory[i].w_z ;
    }
}

void RvizInterfaceNode::control_callback(const holohover_msgs::msg::HolohoverControlStamped &control)
{
    current_control(0) = control.motor_a_1;
    current_control(1) = control.motor_a_2;
    current_control(2) = control.motor_b_1;
    current_control(3) = control.motor_b_2;
    current_control(4) = control.motor_c_1;
    current_control(5) = control.motor_c_2;
}

void RvizInterfaceNode::ref_callback(const geometry_msgs::msg::Pose2D &pose)
{
    ref = pose;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
