#include "simulator_node.hpp"

#include <box2d/box2d.h>


SimulatorNode::SimulatorNode() :
        Node("simulator"),
        gravity(0.0f, 0.0f),
        world(gravity)
{
    init_box2d_world();
    init_box2d_hovercrafts();
    init_topics();
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

void SimulatorNode::init_box2d_hovercrafts()
{
    // hovercraft shape
    b2CircleShape hoverCraftShape;
    hoverCraftShape.m_radius = 1.0f;        // ToDo

    // ToDo if hovercraft enabled in params

    // hovercraft 0
    b2BodyDef hovercraft0_bodyDef;
    hovercraft0_bodyDef.type = b2_dynamicBody;
    hovercraft0_bodyDef.position.Set(0.0f, 4.5f);               // ToDo get starting positions
    hovercraft0 = world.CreateBody(&hovercraft0_bodyDef);
    hovercraft0->CreateFixture(&hoverCraftShape, 1.0f);               // ToDo non-zero density for dynamic body

    // hovercraft 1
    b2BodyDef hovercraft1_bodyDef;
    hovercraft1_bodyDef.type = b2_dynamicBody;
    hovercraft1_bodyDef.position.Set(0.0f, 4.5f);               // ToDo get starting positions
    hovercraft1 = world.CreateBody(&hovercraft1_bodyDef);
    hovercraft1->CreateFixture(&hoverCraftShape, 1.0f);               // ToDo non-zero density for dynamic body
    
    // hovercraft 2
    b2BodyDef hovercraft2_bodyDef;
    hovercraft2_bodyDef.type = b2_dynamicBody;
    hovercraft2_bodyDef.position.Set(0.0f, 4.5f);               // ToDo get starting positions
    hovercraft2 = world.CreateBody(&hovercraft2_bodyDef);
    hovercraft2->CreateFixture(&hoverCraftShape, 1.0f);               // ToDo non-zero density for dynamic body

    // hovercraft 3
    b2BodyDef hovercraft3_bodyDef;
    hovercraft3_bodyDef.type = b2_dynamicBody;
    hovercraft3_bodyDef.position.Set(0.0f, 4.5f);               // ToDo get starting positions
    hovercraft3 = world.CreateBody(&hovercraft3_bodyDef);
    hovercraft3->CreateFixture(&hoverCraftShape, 1.0f);               // ToDo non-zero density for dynamic body
}

void SimulatorNode::init_topics()
{   
    // Todo parameters check if each hovercraft is enabled

    control_subscription_0 = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(
            "hovercraft0/control", rclcpp::SensorDataQoS(),
            std::bind(&SimulatorNode::control_callback_0, this, std::placeholders::_1));

    control_subscription_1 = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(
            "hovercraft1/control", rclcpp::SensorDataQoS(),
            std::bind(&SimulatorNode::control_callback_1, this, std::placeholders::_1));

    control_subscription_2 = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(
            "hovercraft2/control", rclcpp::SensorDataQoS(),
            std::bind(&SimulatorNode::control_callback_2, this, std::placeholders::_1));

    control_subscription_3 = this->create_subscription<holohover_msgs::msg::HolohoverControlStamped>(
            "hovercraft3/control", rclcpp::SensorDataQoS(),
            std::bind(&SimulatorNode::control_callback_3, this, std::placeholders::_1));
    
    pose_publisher_0 = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "hovercraft0/pose", rclcpp::SensorDataQoS());

    pose_publisher_1 = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "hovercraft1/pose", rclcpp::SensorDataQoS());

    pose_publisher_2 = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "hovercraft2/pose", rclcpp::SensorDataQoS());

    pose_publisher_3 = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "hovercraft3/pose", rclcpp::SensorDataQoS());

}

void SimulatorNode::init_timer()
{
    timer = this->create_wall_timer(
            std::chrono::duration<double>(0.04),                        // ToDo parametrize duration
            std::bind(&SimulatorNode::simulation_step, this));
}

void SimulatorNode::simulation_step()
{
    // - control_* to force/torque to apply for each hovercraft - ToDo copy from old simulation node
    // ....

    // - apply forces to box2d bodies
    // hovercraft0->ApplyForceToCenter();
    // hovercraft0->ApplyTorque(0.0f);

    // hovercraft1->ApplyForceToCenter();
    // hovercraft0->ApplyTorque(0.0f);

    // hovercraft2->ApplyForceToCenter();
    // hovercraft0->ApplyTorque(0.0f);

    // hovercraft3->ApplyForceToCenter();
    // hovercraft0->ApplyTorque(0.0f);


    // - box2d step of the world

    float timeStep = 1.0f / 60.0f;              // simulating at 60Hz - ToDo parametrize

    // number of internal iterations
    int32 velocityIterations = 6;               // ToDo parametrize
    int32 positionIterations = 2;
    world.Step(timeStep, velocityIterations, positionIterations);

    // - get position and orientation of each hovercraft from box2d
    b2Vec2 position0 = hovercraft0->GetPosition();
    float angle0 = hovercraft0->GetAngle();

    b2Vec2 position1 = hovercraft1->GetPosition();
    float angle1 = hovercraft1->GetAngle();

    b2Vec2 position2 = hovercraft2->GetPosition();
    float angle2 = hovercraft2->GetAngle();

    b2Vec2 position3 = hovercraft3->GetPosition();
    float angle3 = hovercraft3->GetAngle();

    // - publish pose of each hovercraft
    // pose_publisher_0->publish(..);

    /* from old simulation node - ToDo
    
    geometry_msgs::msg::PoseStamped pose_measurement;
    pose_measurement.header.frame_id = "world";
    pose_measurement.header.stamp = this->now();
    pose_measurement.pose.position.x = state(0) + simulation_settings.Gx;
    pose_measurement.pose.position.y = state(1) + simulation_settings.Gy;
    pose_measurement.pose.position.z = 0;
    double theta = state(4);

    std::normal_distribution<> pose_x_noise(0, simulation_settings.sensor_pose_noise_x);
    std::normal_distribution<> pose_y_noise(0, simulation_settings.sensor_pose_noise_y);
    std::normal_distribution<> pose_yaw_noise(0, simulation_settings.sensor_pose_noise_yaw);
    pose_measurement.pose.position.x += pose_x_noise(random_engine);
    pose_measurement.pose.position.y += pose_y_noise(random_engine);
    theta += pose_yaw_noise(random_engine);

    if (theta < -M_PI)
    {
        theta += 2 * M_PI;
    }
    if (theta > M_PI)
    {
        theta -= 2 * M_PI;
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    pose_measurement.pose.orientation.w = q.w();
    pose_measurement.pose.orientation.x = q.x();
    pose_measurement.pose.orientation.y = q.y();
    pose_measurement.pose.orientation.z = q.z();

    pose_publisher->publish(pose_measurement);*/
}

void SimulatorNode::control_callback_0(const holohover_msgs::msg::HolohoverControlStamped &msg) {
    control_0 = msg;
}

void SimulatorNode::control_callback_1(const holohover_msgs::msg::HolohoverControlStamped &msg) {
    control_1 = msg;
}

void SimulatorNode::control_callback_2(const holohover_msgs::msg::HolohoverControlStamped &msg) {
    control_2 = msg;
}

void SimulatorNode::control_callback_3(const holohover_msgs::msg::HolohoverControlStamped &msg) {
    control_3 = msg;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatorNode>());
    rclcpp::shutdown();
    return 0;
}
