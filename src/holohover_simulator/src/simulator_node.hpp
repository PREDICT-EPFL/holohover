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
#include <std_msgs/msg/float64_multi_array.hpp>
#include "simulator_collisions.hpp"

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
    ~SimulatorNode();
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

    std::vector<rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr> control_subscriptions;

    void control_callback(std::shared_ptr<holohover_msgs::msg::HolohoverControlStamped> msg, long int hovercraft_id);

    std::vector<holohover_msgs::msg::HolohoverControlStamped>   control_msgs;
    std::vector<Holohover::state_t<double>>                     states_vec;
    std::vector<Holohover::control_force_t<double>>             motor_velocities_vec; 
    std::vector<Holohover::control_acc_t<double>>               control_acc_vec;
    bool are_all_simulated;

    // Box2D steps per period (default 10: 1 ms at the 10 ms period). Box2D finds contacts at the start of a
    // step, from the positions the previous step left, so an overlap is only resolved one Box2D step after it
    // appears: at 10 ms a late contact normal turned the puck by up to 6 deg
    int substeps;

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

    // ~/reset: (x, y, yaw, v_x, v_y, w_z) of every simulated hovercraft, then (x, y, v_x, v_y[, w_z]) of the puck (w_z: spin, 0 when left out)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr reset_subscription;
    void reset_callback(const std_msgs::msg::Float64MultiArray &msg);

    // Update Sep 30: collision detector
    std::shared_ptr<ContactListener> contact_listener_; // UPDATE SEP 10: collision detector

};

#endif //HOLOHOVER_UTILS_SIMULATOR_NODE_HPP
