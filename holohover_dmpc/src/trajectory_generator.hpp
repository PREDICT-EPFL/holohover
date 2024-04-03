#ifndef HOLOHOVER_DMPC_TRAJECTORY_GENERATOR_HPP
#define HOLOHOVER_DMPC_TRAJECTORY_GENERATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "holohover_msgs/msg/holohover_dmpc_state_ref_stamped.hpp"
#include "yaml-cpp/yaml.h"

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

    void runTask();
private:
    std::string filename;

    YAML::Node config;

    GeneralConfig gc;

    std::map<int, rclcpp::Publisher<holohover_msgs::msg::HolohoverDmpcStateRefStamped>::SharedPtr> publishers;

    std::shared_ptr<rclcpp::Rate> rate;

    GeneralConfig parseGeneralConfig(YAML::Node& config);

    inline bool coord_updated(Coordinates a, Coordinates b)
    {
        return (a.x != b.x || a.y != b.y || a.yaw != b.yaw);
    }
};

#endif  // HOLOHOVER_DMPC_TRAJECTORY_GENERATOR_HPP
