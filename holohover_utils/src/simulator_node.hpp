#ifndef HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP
#define HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_msgs/msg/holohover_trajectory.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "simulation_settings.hpp"
#include <tf2/LinearMath/Quaternion.h>



#include <box2d/box2d.h>


class SimulatorNode : public rclcpp::Node
{
public:
    SimulatorNode();
private:
    HolohoverProps holohover_props;
    SimulationSettings simulation_settings;

    Holohover holohover;

    b2Vec2 gravity;

    b2World world;

    float timeStep;
    int32 velocityIterations;
    int32 positionIterations;

    std::vector<b2Body*> hovercraft_bodies;
    b2CircleShape hovercraft_shape;

    rclcpp::TimerBase::SharedPtr timer;

    std::vector<long int> hovercraft_ids;

    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers;
    
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

    void calculate_control_acc(Holohover::state_t<double> state, Holohover::control_force_t<double> motor_velocities, Holohover::control_acc_t<double> &current_control_acc);

    void body_to_state(Holohover::state_t<double> &state, b2Body* body);
    void apply_control_acc(b2Body* body, Holohover::control_acc_t<double> control_acc);
};

#endif //HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP
