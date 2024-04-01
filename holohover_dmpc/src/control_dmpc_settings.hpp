#ifndef HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP
#define HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP

#include "rclcpp/rclcpp.hpp"

struct ControlDMPCSettings
{
    double period; //seconds
    double rho;
    unsigned int maxiter;

    unsigned int nx; //state dimension
    unsigned int nu; //input dimension
    unsigned int nxd; //dimension of setpoint
    unsigned int N; //horizon
    int idx_eqx0; //equality constraint for initial condition
    int idx_equ0; //equality constraint for u0
    int idx_u0;
    int idx_u1;
    int idx_x0;

    int Nagents;
    int my_id;
    std::string folder_name_sprob;
};

ControlDMPCSettings load_control_dmpc_settings(rclcpp::Node &node)
{
    ControlDMPCSettings settings;
    
    settings.period = node.declare_parameter<double>("period");
    settings.rho = node.declare_parameter<double>("rho");
    settings.maxiter = node.declare_parameter<int>("maxiter"); 

    settings.nx = node.declare_parameter<int>("nx");
    settings.nu = node.declare_parameter<int>("nu");
    settings.nxd = node.declare_parameter<int>("nxd");
    settings.N = node.declare_parameter<int>("N");
    settings.idx_eqx0 = node.declare_parameter<int>("idx_eqx0");
    settings.idx_equ0 = node.declare_parameter<int>("idx_equ0");
    settings.idx_u0 = node.declare_parameter<int>("idx_u0");
    settings.idx_u1 = node.declare_parameter<int>("idx_u1");
    settings.idx_x0 = node.declare_parameter<int>("idx_x0");

    settings.Nagents = node.declare_parameter<int>("Nagents");
    settings.my_id = node.declare_parameter<int>("my_id");
    settings.folder_name_sprob = node.declare_parameter<std::string>("folder_name_sprob");   

    return settings;
}

#endif //HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP