#include "trajectory_generator.hpp"

TrajectoryGenerator::TrajectoryGenerator() : 
    Node("trajectory_generator"),
    names(declare_parameter<std::vector<std::string>>("names")),
    ids(declare_parameter<std::vector<long int>>("ids"))
{
    for(const auto& id : ids)
    {
        auto topic_name = "/" + names[id] + "/dmpc_state_ref";
        publishers[id] = this->create_publisher<holohover_msgs::msg::HolohoverDmpcStateRefStamped>(topic_name, 10);
    }

    service = create_service<holohover_msgs::srv::TrajectoryGeneratorTrigger>("trajectory_generator", std::bind(&TrajectoryGenerator::runTask, this,
                                std::placeholders::_1, std::placeholders::_2));
}


void TrajectoryGenerator::runTask(const std::shared_ptr<holohover_msgs::srv::TrajectoryGeneratorTrigger::Request>  request,
                                  const std::shared_ptr<holohover_msgs::srv::TrajectoryGeneratorTrigger::Response> response) {
    (void)response;
    
    std::string filename = request->filename.data;
    YAML::Node config = YAML::LoadFile(filename);
    GeneralConfig gc = parseGeneralConfig(config);

    rate = std::make_shared<rclcpp::Rate>(1.0/gc.time_step);

    Step s;
    for (const auto& step : config["trajectory"])
    {
        if(!rclcpp::ok())
        {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Trajectory step...");
        RCLCPP_INFO(this->get_logger(), "\tID\tx\t\ty\t\tyaw");

        for (const auto& element : step["step"])
        {
            int id = element["id"].as<int>();

            Coordinates c;
            c.updated = true;
            
            c.x   = element["x"].as<double>();
            c.y   = element["y"].as<double>();
            c.yaw = element["yaw"].as<double>();

            RCLCPP_INFO(this->get_logger(), "\t%d\t%f\t%f\t%f", id, c.x, c.y, c.yaw);

            s.coord[id] = c;
        }

        for(const auto& id : gc.ids)
        {
            
            holohover_msgs::msg::HolohoverDmpcStateRefStamped msg;

            msg.header.stamp = this->now();

            msg.val_length = 6;
            msg.ref_value.push_back(s.coord[id].x);
            msg.ref_value.push_back(s.coord[id].y);
            msg.ref_value.push_back(0.0);
            msg.ref_value.push_back(0.0);
            msg.ref_value.push_back(s.coord[id].yaw);
            msg.ref_value.push_back(0.0);

            for(const auto& neighbor : gc.neighbors[id])
            {
                msg.ref_value.push_back(s.coord[neighbor].x);
                msg.ref_value.push_back(s.coord[neighbor].y);
                msg.ref_value.push_back(0.0);
                msg.ref_value.push_back(0.0);
                msg.ref_value.push_back(s.coord[neighbor].yaw);
                msg.ref_value.push_back(0.0);

                msg.val_length += 6;
            }

            publishers[id]->publish(msg);
        }

        rate->sleep();
    }
}

GeneralConfig TrajectoryGenerator::parseGeneralConfig(YAML::Node& config)
{
    GeneralConfig gc;

    gc.time_step = config["time_step"].as<double>();
    gc.names     = config["names"].as<std::vector<std::string>>();
    gc.ids       = config["ids"].as<std::vector<int>>();

    for (const auto& neighbor : config["neighbors"]) 
    {
        for (const auto& pair : neighbor)
        {
            int id = pair.first.as<int>();
            gc.neighbors[id] = pair.second.as<std::vector<int>>();
        }
    }

    return gc;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
