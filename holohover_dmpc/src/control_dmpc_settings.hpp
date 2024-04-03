#ifndef HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP
#define HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


struct ControlDMPCSettings
{
    double dmpc_period; //seconds
    double lqr_period; //seconds
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
    
    settings.dmpc_period = node.declare_parameter<double>("dmpc_period");
    settings.lqr_period = node.declare_parameter<double>("lqr_period");
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
    settings.folder_name_sprob = ament_index_cpp::get_package_share_directory("holohover_dmpc") + "/ocp_specs/" + node.declare_parameter<std::string>("folder_name_sprob");

    return settings;
}

#endif //HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP