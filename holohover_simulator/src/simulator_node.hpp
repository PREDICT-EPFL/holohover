#ifndef HOLOHOVER_UTILS_SIMULATOR_NODE_HPP
#define HOLOHOVER_UTILS_SIMULATOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_msgs/msg/holohover_trajectory.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "holohover_common/utils/simulation_settings.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_msgs/msg/holohover_trajectory.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <box2d/box2d.h>
#include <random>
#include <std_msgs/msg/bool.hpp>

// Update Sep 30: collision detector
class ContactListener : public b2ContactListener {
public:

    ContactListener(rclcpp::Node* node) : node_(node) {
        // Initialize publishers
        wall_collided_pub_ = node_->create_publisher<std_msgs::msg::Bool>("wall_collided", 10);
        hovercraft_collided_pub_ = node_->create_publisher<std_msgs::msg::Bool>("hovercraft_collided", 10);

    }

    void BeginContact(b2Contact* contact) override {
        // std::cout << "Collision started!" << std::endl << std::flush;
        // RCLCPP_INFO(logger, "Collision started!");
        auto* bodyA = contact->GetFixtureA()->GetBody();
        auto* bodyB = contact->GetFixtureB()->GetBody();

        // Retrieve user data correctly as uintptr_t and cast to const char*
        const char* dataA = reinterpret_cast<const char*>(bodyA->GetUserData().pointer);
        const char* dataB = reinterpret_cast<const char*>(bodyB->GetUserData().pointer);

        std_msgs::msg::Bool msg;
        // Check for puck collisions
        if ((dataA && std::string(dataA) == "puck") || (dataB && std::string(dataB) == "puck")) {
            // Determine what the puck collided with
            if ((dataA && std::string(dataA) == "hovercraft") || (dataB && std::string(dataB) == "hovercraft")) {
                RCLCPP_INFO(node_->get_logger(), "Puck collided with hovercraft.");
                msg.data = true;
                hovercraft_collided_pub_->publish(msg);
            } else if ((dataA && std::string(dataA) == "wall") || (dataB && std::string(dataB) == "wall")) {
                RCLCPP_INFO(node_->get_logger(), "Puck collided with wall.");
                msg.data = true;
                wall_collided_pub_->publish(msg); 
            } else {
                RCLCPP_WARN(node_->get_logger(), "Puck collided with an unknown object.");
            }
        }
    }

    void EndContact(b2Contact* contact) override {
        // std::cout << "Collision ended!" << std::endl << std::flush;
        // RCLCPP_INFO(logger, "Collision ended!");
        auto* bodyA = contact->GetFixtureA()->GetBody();
        auto* bodyB = contact->GetFixtureB()->GetBody();

        // Retrieve user data correctly as uintptr_t and cast to const char*
        const char* dataA = reinterpret_cast<const char*>(bodyA->GetUserData().pointer);
        const char* dataB = reinterpret_cast<const char*>(bodyB->GetUserData().pointer);

        std_msgs::msg::Bool msg;
        // Check for puck collisions
        if ((dataA && std::string(dataA) == "puck") || (dataB && std::string(dataB) == "puck")) {
            // Determine what the puck stopped colliding with
            if ((dataA && std::string(dataA) == "hovercraft") || (dataB && std::string(dataB) == "hovercraft")) {
                RCLCPP_INFO(node_->get_logger(), "Puck stopped colliding with hovercraft.");
                msg.data = false;
                hovercraft_collided_pub_->publish(msg);
            } else if ((dataA && std::string(dataA) == "wall") || (dataB && std::string(dataB) == "wall")) {
                RCLCPP_INFO(node_->get_logger(), "Puck stopped colliding with wall.");
                msg.data = false;
                wall_collided_pub_->publish(msg); 
            } else {
                RCLCPP_WARN(node_->get_logger(), "Puck stopped colliding with an unknown object.");
            }
        }
    }
private:
    rclcpp::Node* node_;  // Use raw pointer
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wall_collided_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hovercraft_collided_pub_;
};


class BodyDeleter
{
public:
    BodyDeleter(std::shared_ptr<b2World> world) : m_world(world) {}
    void operator()(b2Body* body)
    {
        if (m_world) {
            m_world->DestroyBody(body);
        }
    }
private:
    std::shared_ptr<b2World> m_world;
};

using body_ptr = std::unique_ptr<b2Body, BodyDeleter>;

// Update Sep 30: collision detector
class SimulatorNode : public rclcpp::Node, public std::enable_shared_from_this<SimulatorNode>
{
public:
    SimulatorNode();
private:
    SimulationSettings simulation_settings;
    
    std::vector<HolohoverProps> holohover_props_vec;

    std::vector<Holohover> holohover_vec;

    b2Vec2 gravity;
    std::shared_ptr<b2World> world;

    std::vector<body_ptr> hovercraft_bodies;

    rclcpp::TimerBase::SharedPtr table_timer, simulation_timer;

    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr table_pose_publisher;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr table_pose_subscription;
    void table_callback(const geometry_msgs::msg::PoseStamped &raw_pose);
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_publisher;

    std::vector<rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr> control_subscriptions;

    void control_callback(std::shared_ptr<holohover_msgs::msg::HolohoverControlStamped> msg, long int hovercraft_id);

    std::vector<holohover_msgs::msg::HolohoverControlStamped>   control_msgs;
    std::vector<Holohover::state_t<double>>                     states_vec;
    std::vector<Holohover::control_force_t<double>>             motor_velocities_vec; 
    std::vector<Holohover::control_acc_t<double>>               control_acc_vec;

    void init_hovercraft();
    void init_timer();
    void init_box2d_world();
    void simulation_step();
    void table_publisher();

    void calculate_control_acc(Holohover::state_t<double> state, Holohover::control_force_t<double> motor_velocities, Holohover::control_acc_t<double> &current_control_acc, int i);

    void body_to_state(Holohover::state_t<double> &state, body_ptr &body);
    void apply_control_acc(body_ptr &body, Holohover::control_acc_t<double> control_acc, int i);

    // Update Sep 30: puck simulation
    Holohover::state_t<double>                                  state_puck;
    std::vector<body_ptr>                                       puck_bodies;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr puck_pose_publisher;
    void init_puck();

    // Update Sep 30: collision detector
    std::shared_ptr<ContactListener> contact_listener_; // UPDATE SEP 10: collision detector

};

#endif //HOLOHOVER_UTILS_SIMULATOR_NODE_HPP
