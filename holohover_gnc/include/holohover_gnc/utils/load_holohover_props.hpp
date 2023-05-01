#ifndef HOLOHOVER_GNC_HOLOHOVER_LOAD_HOLOHOVER_PROPS_HPP
#define HOLOHOVER_GNC_HOLOHOVER_LOAD_HOLOHOVER_PROPS_HPP

#include "rclcpp/rclcpp.hpp"
#include "holohover_gnc/models/holohover_model.hpp"

HolohoverProps load_holohover_pros(rclcpp::Node &node)
{
    HolohoverProps props;

    if (node.get_parameter("propeller_pair_gap_distance", props.propeller_pair_gap_distance) &&
        node.get_parameter("propeller_pair_radial_distance", props.propeller_pair_radial_distance) &&
        node.get_parameter("phi_offset", props.phi_offset) &&
        node.get_parameter("mass", props.mass) &&
        node.get_parameter("CoM", props.CoM) &&
        node.get_parameter("inertia", props.inertia) &&
        node.get_parameter("max_thrust", props.max_thrust) &&
        node.get_parameter("signal_to_thrust_coeffs_motor1", props.signal_to_thrust_coeffs_motor1) &&
        node.get_parameter("thrust_to_signal_coeffs_motor1", props.thrust_to_signal_coeffs_motor1) &&
        node.get_parameter("signal_to_thrust_coeffs_motor2", props.signal_to_thrust_coeffs_motor2) &&
        node.get_parameter("thrust_to_signal_coeffs_motor2", props.thrust_to_signal_coeffs_motor2) &&
        node.get_parameter("signal_to_thrust_coeffs_motor3", props.signal_to_thrust_coeffs_motor3) &&
        node.get_parameter("thrust_to_signal_coeffs_motor3", props.thrust_to_signal_coeffs_motor3) &&
        node.get_parameter("signal_to_thrust_coeffs_motor4", props.signal_to_thrust_coeffs_motor4) &&
        node.get_parameter("thrust_to_signal_coeffs_motor4", props.thrust_to_signal_coeffs_motor4) &&
        node.get_parameter("signal_to_thrust_coeffs_motor5", props.signal_to_thrust_coeffs_motor5) &&
        node.get_parameter("thrust_to_signal_coeffs_motor5", props.thrust_to_signal_coeffs_motor5) &&
        node.get_parameter("signal_to_thrust_coeffs_motor6", props.signal_to_thrust_coeffs_motor6) &&
        node.get_parameter("thrust_to_signal_coeffs_motor6", props.thrust_to_signal_coeffs_motor6) &&
        node.get_parameter("motor_pos_a_1", props.motor_pos_a_1) &&
        node.get_parameter("motor_pos_a_1", props.motor_pos_a_2) &&
        node.get_parameter("motor_pos_b_1", props.motor_pos_b_1) &&
        node.get_parameter("motor_pos_b_2", props.motor_pos_b_2) &&
        node.get_parameter("motor_pos_c_1", props.motor_pos_c_1) &&
        node.get_parameter("motor_pos_c_2", props.motor_pos_c_2) &&
        node.get_parameter("learned_motor_vec_a_1", props.learned_motor_vec_a_1) &&
        node.get_parameter("learned_motor_vec_a_2", props.learned_motor_vec_a_2) &&
        node.get_parameter("learned_motor_vec_b_1", props.learned_motor_vec_b_1) &&
        node.get_parameter("learned_motor_vec_b_2", props.learned_motor_vec_b_2) &&
        node.get_parameter("learned_motor_vec_c_1", props.learned_motor_vec_c_1) &&
        node.get_parameter("learned_motor_vec_c_2", props.learned_motor_vec_c_2) 
        )
    {
        props.angle_propeller_pair = atan2(props.propeller_pair_gap_distance / 2, props.propeller_pair_radial_distance);
        props.radius_propeller = sqrt(props.propeller_pair_radial_distance * props.propeller_pair_radial_distance + props.propeller_pair_gap_distance * props.propeller_pair_gap_distance / 4);


        // props.radius_propeller_a_1 = sqrt((props.CoM[1]-props.motor_pos_a_1[1])*(props.CoM[1]-props.motor_pos_a_1[1])+(props.CoM[2]-props.motor_pos_a_1[2])*(props.CoM[2]-props.motor_pos_a_1[2]));
        // props.radius_propeller_a_2 = sqrt((props.CoM[1]-props.motor_pos_a_2[1])*(props.CoM[1]-props.motor_pos_a_2[1])+(props.CoM[2]-props.motor_pos_a_2[2])*(props.CoM[2]-props.motor_pos_a_2[2]));

        // props.radius_propeller_b_1 = sqrt((props.CoM[1]-props.motor_pos_b_1[1])*(props.CoM[1]-props.motor_pos_b_1[1])+(props.CoM[2]-props.motor_pos_b_1[2])*(props.CoM[2]-props.motor_pos_b_1[2]));
        // props.radius_propeller_b_2 = sqrt((props.CoM[1]-props.motor_pos_b_2[1])*(props.CoM[1]-props.motor_pos_b_2[1])+(props.CoM[2]-props.motor_pos_b_2[2])*(props.CoM[2]-props.motor_pos_b_2[2]));
        
        // props.radius_propeller_c_1 = sqrt((props.CoM[1]-props.motor_pos_c_1[1])*(props.CoM[1]-props.motor_pos_c_1[1])+(props.CoM[2]-props.motor_pos_c_1[2])*(props.CoM[2]-props.motor_pos_c_1[2]));
        // props.radius_propeller_c_2 = sqrt((props.CoM[1]-props.motor_pos_c_2[1])*(props.CoM[1]-props.motor_pos_c_2[1])+(props.CoM[2]-props.motor_pos_c_2[2])*(props.CoM[2]-props.motor_pos_c_2[2]));

    
    }
    else
    {
        RCLCPP_INFO(node.get_logger(), "Failed to load holohover parameters");
    }

    return props;
}

#endif //HOLOHOVER_GNC_HOLOHOVER_LOAD_HOLOHOVER_PROPS_HPP
