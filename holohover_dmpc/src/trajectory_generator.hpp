#ifndef HOLOHOVER_DMPC_TRAJECTORY_GENERATOR_HPP
#define HOLOHOVER_DMPC_TRAJECTORY_GENERATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "holohover_msgs/msg/holohover_dmpc_state_ref_stamped.hpp"
#include "holohover_msgs/msg/holohover_state.hpp"
#include "yaml-cpp/yaml.h"
#include <thread>

struct GeneralConfig {
    double time_step;
    std::vector<std::string> names;
    std::vector<int> ids;
    std::map<int,std::vector<int>> neighbors;
};

struct Coordinates {
    double x,y,yaw;
    bool updated = true;
};

struct Step {
    std::map<int, Coordinates> coord;
};

class TrajectoryGenerator : public rclcpp::Node
{
public:
    TrajectoryGenerator();

private:
    void dmpcGenerator(GeneralConfig& gc, YAML::Node& config);
    void obstGenerator(GeneralConfig& gc, YAML::Node& config);

    std::vector<std::string> names;
    std::vector<long int> ids;
    std::map<int, rclcpp::Publisher<holohover_msgs::msg::HolohoverDmpcStateRefStamped>::SharedPtr> dmpc_publishers;
    std::map<int, rclcpp::Publisher<holohover_msgs::msg::HolohoverState>::SharedPtr> obst_publishers;

    GeneralConfig parseGeneralConfig(YAML::Node& config);

    inline bool coord_updated(Coordinates a, Coordinates b)
    {
        return (a.x != b.x || a.y != b.y || a.yaw != b.yaw);
    }
};

#endif  // HOLOHOVER_DMPC_TRAJECTORY_GENERATOR_HPP
