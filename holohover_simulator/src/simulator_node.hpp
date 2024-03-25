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

class SimulatorNode : public rclcpp::Node
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

    rclcpp::TimerBase::SharedPtr simulation_timer;

    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_publisher;

    std::vector<rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr> control_subscriptions;

    void control_callback(std::shared_ptr<holohover_msgs::msg::HolohoverControlStamped> msg, long int hovercraft_id);

    std::vector<holohover_msgs::msg::HolohoverControlStamped>   control_msgs;
    std::vector<Holohover::state_t<double>>                     states_vec;
    std::vector<Holohover::control_force_t<double>>             motor_velocities_vec; 
    std::vector<Holohover::control_acc_t<double>>               control_acc_vec;

    void init_hovercrafts();
    void init_timer();
    void init_box2d_world();
    void simulation_step();

    void calculate_control_acc(Holohover::state_t<double> state, Holohover::control_force_t<double> motor_velocities, Holohover::control_acc_t<double> &current_control_acc, int i);

    void body_to_state(Holohover::state_t<double> &state, body_ptr &body);
    void apply_control_acc(body_ptr &body, Holohover::control_acc_t<double> control_acc, int i);
};

#endif //HOLOHOVER_UTILS_SIMULATOR_NODE_HPP
