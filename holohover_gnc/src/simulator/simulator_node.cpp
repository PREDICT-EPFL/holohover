#include "simulator_node.hpp"

SimulatorNode::SimulatorNode() :
    Node("simulator"),
    holohover_props(load_holohover_pros(declare_parameter<std::vector<std::string>>("holohover_props_files")[0])),
    simulation_settings(load_simulation_settings(*this)),
    holohover(holohover_props, simulation_settings.period),
    gravity(0.0f, 0.0f),
    world(std::make_unique<b2World>(gravity))
{
    init_box2d_world(); // ToDo set positions and sizes of walls

    viz_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/drone", 10);

    init_hovercrafts();
    init_timer();
}

void SimulatorNode::init_box2d_world()
{
    b2PolygonShape wallBox;
    b2BodyDef wallDef;
    b2Body *wall;
    
    // Wall 1
    wallBox.SetAsBox(simulation_settings.table_size[0], 0.001);
    wallDef.position.Set(simulation_settings.table_size[0] / 2, 0);
    wall = world->CreateBody(&wallDef);
    wall->CreateFixture(&wallBox, 0.0f); // 0 density for static body

    // Wall 2
    wallBox.SetAsBox(0.001, simulation_settings.table_size[1]);
    wallDef.position.Set(simulation_settings.table_size[0], simulation_settings.table_size[1] / 2);
    wall = world->CreateBody(&wallDef);
    wall->CreateFixture(&wallBox, 0.0f); // 0 density for static body

    // Wall 3
    wallBox.SetAsBox(simulation_settings.table_size[0], 0.001);
    wallDef.position.Set(simulation_settings.table_size[0] / 2, simulation_settings.table_size[1]);
    wall = world->CreateBody(&wallDef);
    wall->CreateFixture(&wallBox, 0.0f); // 0 density for static body

    // Wall 4
    wallBox.SetAsBox(0.001, simulation_settings.table_size[1]);
    wallDef.position.Set(0, simulation_settings.table_size[1] / 2);
    wall = world->CreateBody(&wallDef);
    wall->CreateFixture(&wallBox, 0.0f); // 0 density for static body
    


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

    p1.x = simulation_settings.table_size[0];
    p1.y = 0;
    marker.points.push_back(p1);

    p1.x = simulation_settings.table_size[0];
    p1.y = simulation_settings.table_size[1];
    marker.points.push_back(p1);

    p1.x = 0;
    p1.y = simulation_settings.table_size[1];
    marker.points.push_back(p1);

    p1.x = 0;
    p1.y = 0;
    marker.points.push_back(p1);

    wall_markers.markers.push_back(marker);
}

void SimulatorNode::init_hovercrafts()
{
    b2CircleShape hovercraft_shape;
    hovercraft_shape.m_radius = simulation_settings.hovercraft_radius;
    double density = holohover_props.mass / (M_PI * hovercraft_shape.m_radius * hovercraft_shape.m_radius);  // ToDo density parameter

    for(size_t i = 0; i < simulation_settings.hovercraft_ids.size(); i++) {
        RCLCPP_INFO(get_logger(), "Hovercraft id: %ld", simulation_settings.hovercraft_ids[i]);
        
        //box2d bodies
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(simulation_settings.start_position_x[i], simulation_settings.start_position_y[i]);
        bodyDef.angle = simulation_settings.start_position_theta[i];
        bodyDef.linearVelocity = b2Vec2(simulation_settings.start_position_vx[i], simulation_settings.start_position_vy[i]);
        bodyDef.angularVelocity = simulation_settings.start_position_w[i];
        body_ptr body(world->CreateBody(&bodyDef), world);
        body->CreateFixture(&hovercraft_shape, density);
        hovercraft_bodies.push_back(std::move(body));

        // state
        Holohover::state_t<double> state;
        body_to_state(state, hovercraft_bodies.back());
        states_vec.push_back(state);

        // motor velociticies
        Holohover::control_force_t<double> motor_velocities;
        motor_velocities.setZero();
        motor_velocities_vec.push_back(motor_velocities);

        // control acc
        Holohover::control_acc_t<double> current_control_acc;
        calculate_control_acc(state, motor_velocities, current_control_acc);
        control_acc_vec.push_back(current_control_acc);

        // control messages
        holohover_msgs::msg::HolohoverControlStamped msg;
        // init zero control
        msg.header.frame_id = "body";
        msg.header.stamp = this->now();
        msg.motor_a_1 = 0;
        msg.motor_a_2 = 0;
        msg.motor_b_1 = 0;
        msg.motor_b_2 = 0;
        msg.motor_c_1 = 0;
        msg.motor_c_2 = 0;
        control_msgs.push_back(msg);

        // control subscriptions
        auto topic_name = "/hovercraft" + std::to_string(simulation_settings.hovercraft_ids[i]) + "/control";

        std::function<void(const holohover_msgs::msg::HolohoverControlStamped::SharedPtr)> callback = 
                 std::bind(&SimulatorNode::control_callback, this, std::placeholders::_1, i);

        auto sub = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(topic_name, rclcpp::SensorDataQoS(), callback);

        control_subscriptions.push_back(sub);

        // pose publishers
        topic_name = "/hovercraft" + std::to_string(simulation_settings.hovercraft_ids[i]) + "/pose";
        pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic_name, rclcpp::SensorDataQoS()));

    }
    
}

void SimulatorNode::init_timer()
{
    simulation_timer = this->create_wall_timer(
            std::chrono::duration<double>(simulation_settings.period),
            std::bind(&SimulatorNode::simulation_step, this));

    wall_timer = this->create_wall_timer(
            std::chrono::duration<double>(1),
            std::bind(&SimulatorNode::publish_wall, this));


}
void SimulatorNode::publish_wall()
{
    viz_publisher->publish(wall_markers);
}

void SimulatorNode::simulation_step()
{
    for(size_t i = 0; i < simulation_settings.hovercraft_ids.size(); i++) {
        // integrate motor velocities
        Holohover::control_force_t<double> current_control_signal;
        current_control_signal(0) = control_msgs[i].motor_a_1;
        current_control_signal(1) = control_msgs[i].motor_a_2;
        current_control_signal(2) = control_msgs[i].motor_b_1;
        current_control_signal(3) = control_msgs[i].motor_b_2;
        current_control_signal(4) = control_msgs[i].motor_c_1;
        current_control_signal(5) = control_msgs[i].motor_c_2;
        motor_velocities_vec[i] = holohover.Ad_motor * motor_velocities_vec[i] + holohover.Bd_motor * current_control_signal;
        calculate_control_acc(states_vec[i], motor_velocities_vec[i], control_acc_vec[i]);
        apply_control_acc(hovercraft_bodies[i], control_acc_vec[i]);
        
    }

    // - box2d step of the world
    world->Step(simulation_settings.period, 
                simulation_settings.internal_iterations_velocity, 
                simulation_settings.internal_iterations_position);

    for(size_t i = 0; i < simulation_settings.hovercraft_ids.size(); i++) {
        body_to_state(states_vec[i], hovercraft_bodies[i]);
       
        geometry_msgs::msg::PoseStamped pose_measurement;
        pose_measurement.header.frame_id = "world";
        pose_measurement.header.stamp = this->now();
        pose_measurement.pose.position.x = states_vec[i](0) + simulation_settings.Gx;
        pose_measurement.pose.position.y = states_vec[i](1) + simulation_settings.Gy;
        pose_measurement.pose.position.z = 0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, states_vec[i](4));
        pose_measurement.pose.orientation.w = q.w();
        pose_measurement.pose.orientation.x = q.x();
        pose_measurement.pose.orientation.y = q.y();
        pose_measurement.pose.orientation.z = q.z();
        
        pose_publishers[i]->publish(pose_measurement);
    }
}

void SimulatorNode::control_callback(holohover_msgs::msg::HolohoverControlStamped::SharedPtr msg, long int hovercraft_id) {
    control_msgs[hovercraft_id] = *msg;
}

void SimulatorNode::body_to_state(Holohover::state_t<double> &state, body_ptr &body) {
    // ToDo add noise 
    state(0) = body->GetPosition().x;
    state(1) = body->GetPosition().y;
    state(2) = body->GetLinearVelocity().x;
    state(3) = body->GetLinearVelocity().y;
    state(4) = body->GetAngle();
    state(5) = body->GetAngularVelocity();
}

void SimulatorNode::apply_control_acc(body_ptr &body, Holohover::control_acc_t<double> control_acc) {
    body->ApplyForceToCenter(b2Vec2(control_acc(0) * holohover_props.mass, control_acc(1) * holohover_props.mass), true);
    body->ApplyTorque(control_acc(2) * holohover_props.inertia, true);
}

void SimulatorNode::calculate_control_acc(Holohover::state_t<double> state, Holohover::control_force_t<double> motor_velocities, Holohover::control_acc_t<double> &current_control_acc)
{
    Holohover::control_force_t<double> current_control_force;
    holohover.signal_to_thrust(motor_velocities, current_control_force);
    holohover.control_force_to_acceleration(state, current_control_force, current_control_acc);

    // add drag force
    double rho = 1.2; // air density at ~20C sea level
    current_control_acc(0) -= 0.5 / holohover_props.mass * rho * state(2) * abs(state(2)) * simulation_settings.drag_coefficient * simulation_settings.drag_reference_area;
    current_control_acc(1) -= 0.5 / holohover_props.mass * rho * state(3) * abs(state(3)) * simulation_settings.drag_coefficient * simulation_settings.drag_reference_area;

    // add force introduced by table tilt
    double g = 9.81;
    current_control_acc(0) += sin(simulation_settings.table_tilt_x * M_PI / 180.0) * g;
    current_control_acc(1) += sin(simulation_settings.table_tilt_y * M_PI / 180.0) * g;

    //std::cout << "acc = " << current_control_acc.transpose() << std::endl;
    //std::cout << "force = " << current_control_force.transpose() << std::endl;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatorNode>());
    rclcpp::shutdown();
    return 0;
}
