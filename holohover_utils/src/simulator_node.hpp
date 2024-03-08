#ifndef HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP
#define HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "holohover_msgs/msg/holohover_state_stamped.hpp"
#include "holohover_msgs/msg/holohover_control_stamped.hpp"
#include "holohover_msgs/msg/holohover_trajectory.hpp"
#include "holohover_common/models/holohover_model.hpp"
#include "holohover_common/utils/holohover_props.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <box2d/box2d.h>


class SimulatorNode : public rclcpp::Node
{
public:
    SimulatorNode();
private:
    //b2World world;
    b2Vec2 gravity;//(0.0f, 0.0f);

    b2World world;//(gravity);// = &(new b2World(gravity));

    b2Body *hovercraft0, *hovercraft1, *hovercraft2, *hovercraft3;

    rclcpp::TimerBase::SharedPtr timer;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_0;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_1;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_2;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_3;
    
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_subscription_0;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_subscription_1;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_subscription_2;
    rclcpp::Subscription<holohover_msgs::msg::HolohoverControlStamped>::SharedPtr control_subscription_3;

    void control_callback_0(const holohover_msgs::msg::HolohoverControlStamped &msg);
    void control_callback_1(const holohover_msgs::msg::HolohoverControlStamped &msg);
    void control_callback_2(const holohover_msgs::msg::HolohoverControlStamped &msg);
    void control_callback_3(const holohover_msgs::msg::HolohoverControlStamped &msg);

    holohover_msgs::msg::HolohoverControlStamped control_0;
    holohover_msgs::msg::HolohoverControlStamped control_1;
    holohover_msgs::msg::HolohoverControlStamped control_2;
    holohover_msgs::msg::HolohoverControlStamped control_3;


    void init_topics();
    void init_timer();
    void init_box2d_world();
    void init_box2d_hovercrafts();
    void simulation_step();


};

#endif //HOLOHOVER_UTILS_RVIZ_INTERFACE_NODE_HPP
