#ifndef HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP
#define HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


struct ControlDMPCSettings
{
    double dmpc_period; //seconds
    double rho;
    unsigned int maxiter;
    unsigned int max_outer_iter;
    unsigned int nx; //state dimension
    unsigned int nu; //input dimension
    unsigned int nxd; //dimension of setpoint
    unsigned int nud; //dimension of input setpoint
    unsigned int N; //horizon
    int idx_eqx0; //equality constraint for initial condition
    int idx_equ0; //equality constraint for u0
    int idx_u0;
    int idx_u1;
    int idx_x0;

    int Nagents;
    int my_id;
    std::string folder_name_sprob;
    std::string file_name_xd_trajectory;
    std::string file_name_ud_trajectory;
};

ControlDMPCSettings load_control_dmpc_settings(rclcpp::Node &node)
{
    ControlDMPCSettings settings;
    
    settings.dmpc_period = node.declare_parameter<double>("dmpc_period");
    settings.rho = node.declare_parameter<double>("rho");
    settings.maxiter = node.declare_parameter<int>("maxiter"); 
    settings.max_outer_iter = node.declare_parameter<int>("max_outer_iter");
    settings.nx = node.declare_parameter<int>("nx");
    settings.nu = node.declare_parameter<int>("nu");
    settings.nxd = node.declare_parameter<int>("nxd");
    settings.nud = node.declare_parameter<int>("nud");
    settings.N = node.declare_parameter<int>("N");
    settings.idx_eqx0 = node.declare_parameter<int>("idx_eqx0");
    settings.idx_equ0 = node.declare_parameter<int>("idx_equ0");
    settings.idx_u0 = node.declare_parameter<int>("idx_u0");
    settings.idx_u1 = node.declare_parameter<int>("idx_u1");
    settings.idx_x0 = node.declare_parameter<int>("idx_x0");

    settings.Nagents = node.declare_parameter<int>("Nagents");
    settings.my_id = node.declare_parameter<int>("my_id");
    settings.folder_name_sprob = ament_index_cpp::get_package_share_directory("holohover_dmpc") + "/ocp_specs/" + node.declare_parameter<std::string>("folder_name_sprob");
    
    
    std::string tmp_ = node.declare_parameter<std::string>("file_name_xd_trajectory");
    if (tmp_.empty()){
        settings.file_name_xd_trajectory = "";
    } else{
        settings.file_name_xd_trajectory = ament_index_cpp::get_package_share_directory("holohover_dmpc") + "/config/trajectories/" + tmp_;
    }

    tmp_ = node.declare_parameter<std::string>("file_name_ud_trajectory");
    if (tmp_.empty()){
        settings.file_name_ud_trajectory = "";
    } else{
        settings.file_name_ud_trajectory = ament_index_cpp::get_package_share_directory("holohover_dmpc") + "/config/trajectories/" + tmp_;
    }

    return settings;
}

#endif //HOLOHOVER_GNC_CONTROL_DMPC_SETTINGS_HPP