#ifndef HOLOHOVER_COMMON_HOLOHOVER_PROPS_HPP
#define HOLOHOVER_COMMON_HOLOHOVER_PROPS_HPP

#include "vector"
#include "yaml-cpp/yaml.h"

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
    // first order time constant of motor
    double motor_tau;
    // configuration matrix
    bool use_configuration_matrix;
    std::vector<double> configuration_matrix;
    // polynomial coefficients for signal [0,1] to thrust [N] conversation (coeff of the lowest order polynomial first)
    std::vector<double> signal_to_thrust_coeffs_motor_1;
    std::vector<double> signal_to_thrust_coeffs_motor_2;
    std::vector<double> signal_to_thrust_coeffs_motor_3;
    std::vector<double> signal_to_thrust_coeffs_motor_4;
    std::vector<double> signal_to_thrust_coeffs_motor_5;
    std::vector<double> signal_to_thrust_coeffs_motor_6;
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

HolohoverProps load_holohover_pros(std::string filename)
{
    YAML::Node config = YAML::LoadFile(filename);

    HolohoverProps props;

    props.mass = config["mass"].as<double>();
    
    props.CoM = config["CoM"].as<std::vector<double>>();
    props.inertia = config["inertia"].as<double>();
    props.idle_signal = config["idle_signal"].as<double>();
    props.motor_tau = config["motor_tau"].as<double>();

    props.use_configuration_matrix = config["use_configuration_matrix"].as<bool>();
    props.configuration_matrix = config["configuration_matrix"].as<std::vector<double>>();
    
    props.signal_to_thrust_coeffs_motor_1 = config["signal_to_thrust_coeffs_motor_1"].as<std::vector<double>>();
    props.signal_to_thrust_coeffs_motor_2 = config["signal_to_thrust_coeffs_motor_2"].as<std::vector<double>>();
    props.signal_to_thrust_coeffs_motor_3 = config["signal_to_thrust_coeffs_motor_3"].as<std::vector<double>>();
    props.signal_to_thrust_coeffs_motor_4 = config["signal_to_thrust_coeffs_motor_4"].as<std::vector<double>>();
    props.signal_to_thrust_coeffs_motor_5 = config["signal_to_thrust_coeffs_motor_5"].as<std::vector<double>>();
    props.signal_to_thrust_coeffs_motor_6 = config["signal_to_thrust_coeffs_motor_6"].as<std::vector<double>>();

    props.motor_pos_a_1 = config["motor_pos_a_1"].as<std::vector<double>>();
    props.motor_pos_a_2 = config["motor_pos_a_2"].as<std::vector<double>>();
    props.motor_pos_b_1 = config["motor_pos_b_1"].as<std::vector<double>>();
    props.motor_pos_b_2 = config["motor_pos_b_2"].as<std::vector<double>>();
    props.motor_pos_c_1 = config["motor_pos_c_1"].as<std::vector<double>>();
    props.motor_pos_c_2 = config["motor_pos_c_2"].as<std::vector<double>>();

    props.learned_motor_vec_a_1 = config["learned_motor_vec_a_1"].as<std::vector<double>>();
    props.learned_motor_vec_a_2 = config["learned_motor_vec_a_2"].as<std::vector<double>>();
    props.learned_motor_vec_b_1 = config["learned_motor_vec_b_1"].as<std::vector<double>>();
    props.learned_motor_vec_b_2 = config["learned_motor_vec_b_2"].as<std::vector<double>>();
    props.learned_motor_vec_c_1 = config["learned_motor_vec_c_1"].as<std::vector<double>>();
    props.learned_motor_vec_c_2 = config["learned_motor_vec_c_2"].as<std::vector<double>>();

    return props;
}

#endif //HOLOHOVER_COMMON_HOLOHOVER_PROPS_HPP
