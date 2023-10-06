#ifndef HOLOHOVER_COMMON_HOLOHOVER_PROPS_HPP
#define HOLOHOVER_COMMON_HOLOHOVER_PROPS_HPP

#include "vector"
#include "rclcpp/rclcpp.hpp"

struct HolohoverProps
{
    // mass of the hovercraft
    double mass;
    // Learned CoM 
    std::vector<double> CoM;
    // inertia in the z-direction
    double inertia;
    // minimum idle signal
    double idle_signal;
    // polynomial coefficients for signal [0,1] to thrust [N] conversation (coeff of the highest order polynomial first)
    std::vector<double> signal_to_thrust_coeffs_motor_1;
    std::vector<double> signal_to_thrust_coeffs_motor_2;
    std::vector<double> signal_to_thrust_coeffs_motor_3;
    std::vector<double> signal_to_thrust_coeffs_motor_4;
    std::vector<double> signal_to_thrust_coeffs_motor_5;
    std::vector<double> signal_to_thrust_coeffs_motor_6;
    // polynomial coefficients for thrust [N] to signal [0,1] conversation (coeff of the highest order polynomial first)
    std::vector<double> thrust_to_signal_coeffs_motor_1;
    std::vector<double> thrust_to_signal_coeffs_motor_2;
    std::vector<double> thrust_to_signal_coeffs_motor_3;
    std::vector<double> thrust_to_signal_coeffs_motor_4;
    std::vector<double> thrust_to_signal_coeffs_motor_5;
    std::vector<double> thrust_to_signal_coeffs_motor_6;
    // Position of the motor
    std::vector<double> motor_pos_a_1;
    std::vector<double> motor_pos_a_2;
    std::vector<double> motor_pos_b_1;
    std::vector<double> motor_pos_b_2;
    std::vector<double> motor_pos_c_1;
    std::vector<double> motor_pos_c_2;
    // Learned vectors of the motor
    std::vector<double> learned_motor_vec_a_1;
    std::vector<double> learned_motor_vec_a_2;
    std::vector<double> learned_motor_vec_b_1;
    std::vector<double> learned_motor_vec_b_2;
    std::vector<double> learned_motor_vec_c_1;
    std::vector<double> learned_motor_vec_c_2;
};

HolohoverProps load_holohover_pros(rclcpp::Node &node)
{
    HolohoverProps props;

    props.mass = node.declare_parameter<double>("mass");
    props.CoM = node.declare_parameter<std::vector<double>>("CoM");
    props.inertia = node.declare_parameter<double>("inertia");
    props.idle_signal = node.declare_parameter<double>("idle_signal");

    props.signal_to_thrust_coeffs_motor_1 = node.declare_parameter<std::vector<double>>("signal_to_thrust_coeffs_motor_1");
    props.signal_to_thrust_coeffs_motor_2 = node.declare_parameter<std::vector<double>>("signal_to_thrust_coeffs_motor_2");
    props.signal_to_thrust_coeffs_motor_3 = node.declare_parameter<std::vector<double>>("signal_to_thrust_coeffs_motor_3");
    props.signal_to_thrust_coeffs_motor_4 = node.declare_parameter<std::vector<double>>("signal_to_thrust_coeffs_motor_4");
    props.signal_to_thrust_coeffs_motor_5 = node.declare_parameter<std::vector<double>>("signal_to_thrust_coeffs_motor_5");
    props.signal_to_thrust_coeffs_motor_6 = node.declare_parameter<std::vector<double>>("signal_to_thrust_coeffs_motor_6");

    props.thrust_to_signal_coeffs_motor_1 = node.declare_parameter<std::vector<double>>("thrust_to_signal_coeffs_motor_1");
    props.thrust_to_signal_coeffs_motor_2 = node.declare_parameter<std::vector<double>>("thrust_to_signal_coeffs_motor_2");
    props.thrust_to_signal_coeffs_motor_3 = node.declare_parameter<std::vector<double>>("thrust_to_signal_coeffs_motor_3");
    props.thrust_to_signal_coeffs_motor_4 = node.declare_parameter<std::vector<double>>("thrust_to_signal_coeffs_motor_4");
    props.thrust_to_signal_coeffs_motor_5 = node.declare_parameter<std::vector<double>>("thrust_to_signal_coeffs_motor_5");
    props.thrust_to_signal_coeffs_motor_6 = node.declare_parameter<std::vector<double>>("thrust_to_signal_coeffs_motor_6");

    props.motor_pos_a_1 = node.declare_parameter<std::vector<double>>("motor_pos_a_1");
    props.motor_pos_a_2 = node.declare_parameter<std::vector<double>>("motor_pos_a_2");
    props.motor_pos_b_1 = node.declare_parameter<std::vector<double>>("motor_pos_b_1");
    props.motor_pos_b_2 = node.declare_parameter<std::vector<double>>("motor_pos_b_2");
    props.motor_pos_c_1 = node.declare_parameter<std::vector<double>>("motor_pos_c_1");
    props.motor_pos_c_2 = node.declare_parameter<std::vector<double>>("motor_pos_c_2");

    props.learned_motor_vec_a_1 = node.declare_parameter<std::vector<double>>("learned_motor_vec_a_1");
    props.learned_motor_vec_a_2 = node.declare_parameter<std::vector<double>>("learned_motor_vec_a_2");
    props.learned_motor_vec_b_1 = node.declare_parameter<std::vector<double>>("learned_motor_vec_b_1");
    props.learned_motor_vec_b_2 = node.declare_parameter<std::vector<double>>("learned_motor_vec_b_2");
    props.learned_motor_vec_c_1 = node.declare_parameter<std::vector<double>>("learned_motor_vec_c_1");
    props.learned_motor_vec_c_2 = node.declare_parameter<std::vector<double>>("learned_motor_vec_c_2");

    return props;
}

#endif //HOLOHOVER_COMMON_HOLOHOVER_PROPS_HPP
