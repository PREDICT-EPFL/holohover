#ifndef HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP
#define HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP

#include "rclcpp/rclcpp.hpp"

struct ControlDMPCSettings
{
    double period; //seconds
    double rho;
    double maxiter;

    int nx; //state dimension
    int nu; //input dimension
    int nxd; //dimension of setpoint
    int idx_eqx0; //equality constraint for initial condition
    int idx_equ0; //equality constraint for u0
    int idx_u0;
    int idx_u1;

    int Nagents;
};

ControlMPCSettings load_control_dmpc_settings(rclcpp::Node &node)
{
    ControlDMPCSettings settings;
    
    settings.period = node.declare_parameter<double>("period");
    settings.rho = node.declare_parameter<double>("rho");
    settings.maxiter = node.declare_parameter<double>("maxiter");    

    return settings;
}

#endif //HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP