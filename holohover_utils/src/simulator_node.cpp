#include "simulator_node.hpp"

#include <box2d/box2d.h>


SimulatorNode::SimulatorNode() :
    Node("simulator"),
    holohover_props(load_holohover_pros(*this)),
    simulation_settings(load_simulation_settings(*this)),
    holohover(holohover_props, simulation_settings.period),
    gravity(0.0f, 0.0f),
    world(gravity)
{
    hovercraft_ids = declare_parameter<std::vector<long int>>("hovercrafts");
    hovercraft_shape.m_radius = 1.0f;        // ToDo parameter
    // ToDo density parameter

    timeStep = 1.0f / 60.0f;              // simulating at 60Hz - ToDo parametrize

    // number of internal iterations
    velocityIterations = 6;               // ToDo parametrize
    positionIterations = 2;


    init_box2d_world();
    init_hovercrafts();
    init_timer();
}

void SimulatorNode::init_box2d_world()
{
    // static body - ToDo insert 4 walls - ToDo parametrize position of the walls
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f, -10.0f);
    b2Body* groundBody = world.CreateBody(&groundBodyDef);
    b2PolygonShape groundBox;
    groundBox.SetAsBox(50.0f, 10.0f);
    groundBody->CreateFixture(&groundBox, 0.0f);                    // 0 density for static body
}

void SimulatorNode::init_hovercrafts()
{

    for(long int hovercraft_id : hovercraft_ids) {
        RCLCPP_INFO(get_logger(), "Hovercraft id: %ld", hovercraft_id);
        
        //box2d bodies
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(0.0f, 0.0f);               // ToDo get starting positions
        bodyDef.angle = 0.0f;
        bodyDef.linearVelocity = b2Vec2(0.0f, 0.0f);
        bodyDef.angularVelocity = 0.0f;
        b2Body * body = world.CreateBody(&bodyDef);
        body->CreateFixture(&hovercraft_shape, 1.0f);               // ToDo non-zero density for dynamic body
        hovercraft_bodies.push_back(body);

        // state
        Holohover::state_t<double> state;
        body_to_state(state, body);
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
        auto topic_name = "/hovercraft" + std::to_string(hovercraft_id) + "/control";

        std::function<void(const holohover_msgs::msg::HolohoverControlStamped::SharedPtr)> callback = 
                 std::bind(&SimulatorNode::control_callback, this, std::placeholders::_1, hovercraft_id);

        auto sub = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(topic_name, rclcpp::SensorDataQoS(), callback);

        control_subscriptions.push_back(sub);

        // pose publishers
        topic_name = "/hovercraft" + std::to_string(hovercraft_id) + "/pose";
        pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic_name, rclcpp::SensorDataQoS()));
    }
    
}

void SimulatorNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(0.04),                        // ToDo parametrize duration
            std::bind(&SimulatorNode::simulation_step, this));
}

void SimulatorNode::simulation_step()
{
    for(long int i : hovercraft_ids) {
        // - control_* to force/torque to apply for each hovercraft - ToDo copy from old simulation node
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
    world.Step(timeStep, velocityIterations, positionIterations);

    for(long int i : hovercraft_ids) {
        body_to_state(states_vec[i], hovercraft_bodies[i]);
       
        geometry_msgs::msg::PoseStamped pose_measurement;
        pose_measurement.header.frame_id = "world";
        pose_measurement.header.stamp = this->now();
        pose_measurement.pose.position.x = states_vec[i](0);// ToDo??? + simulation_settings.Gx;
        pose_measurement.pose.position.y = states_vec[i](1);// + simulation_settings.Gy;
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

void SimulatorNode::body_to_state(Holohover::state_t<double> &state, b2Body* body) {
    // ToDo add noise 
    state(0) = body->GetPosition().x;
    state(1) = body->GetPosition().y;
    state(2) = body->GetLinearVelocity().x;
    state(3) = body->GetLinearVelocity().y;
    state(4) = body->GetAngle();
    state(5) = body->GetAngularVelocity();
}

void SimulatorNode::apply_control_acc(b2Body* body, Holohover::control_acc_t<double> control_acc) {
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
