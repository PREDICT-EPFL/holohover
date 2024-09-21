#include "trajectory_generator.hpp"

TrajectoryGenerator::TrajectoryGenerator() : 
    Node("trajectory_generator"),
    names(declare_parameter<std::vector<std::string>>("names")),
    ids(declare_parameter<std::vector<long int>>("ids"))
{
    std::string filename, obst_filename;

    for(const auto& id : ids)
    {
        if (names[id].rfind("h", 0) == 0) { 
            auto topic_name = "/" + names[id] + "/dmpc_state_ref";
            RCLCPP_INFO(this->get_logger(), "Publishing to : %s", topic_name.c_str());

            dmpc_publishers[id] = this->create_publisher<holohover_msgs::msg::HolohoverDmpcStateRefStamped>(topic_name, 10);
        } else if (names[id].rfind("o", 0) == 0){
            auto topic_name = "/" + names[id] + "/state_ref";
            RCLCPP_INFO(this->get_logger(), "Publishing to : %s", topic_name.c_str());

            obst_publishers[id] = this->create_publisher<holohover_msgs::msg::HolohoverState>(topic_name, 10);
        }
    }

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("holohover_dmpc");

    YAML::Node config, obst_config;
    GeneralConfig gc, obst_gc;

    while(true)
    {
        while(true) {
            try {
                std::cout << "Insert the name of the YAML file for DMPC hovercraft: ";
                std::cin >> filename;

                if(filename == "none") {
                    RCLCPP_INFO(this->get_logger(), "No hovercraft trajectory file provided.");
                    break;
                }

                std::string path = package_share_directory + "/config/trajectories/" + filename;
                RCLCPP_INFO(this->get_logger(), "Opening trajectory file: %s", path.c_str());
                config = YAML::LoadFile(path);
                gc = parseGeneralConfig(config);
                break;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error reading YAML file for DMPC. Exception: %s", e.what());
            }
        }

        while(true) {
            std::cout << "Insert the name of the YAML file for LQR obstacles (or none): ";
            std::cin >> obst_filename;

            if(obst_filename == "none") {
                RCLCPP_INFO(this->get_logger(), "No obstacles trajectory file provided.");
                break;
            }

            try {
                std::string path = package_share_directory + "/config/trajectories_obstacles/" + obst_filename;
                RCLCPP_INFO(this->get_logger(), "Opening obstacles trajectory file: %s", path.c_str());
                obst_config = YAML::LoadFile(path);
                obst_gc = parseGeneralConfig(obst_config);
                
                break;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error reading YAML file for obstacles. Exception: %s", e.what());
            }
        }

        std::thread dmpc;
        if (filename != "none")
            dmpc = std::thread(&TrajectoryGenerator::dmpcGenerator, this, std::ref(gc), std::ref(config));
        if (obst_filename != "none") {
            std::thread obst(&TrajectoryGenerator::obstGenerator, this, std::ref(obst_gc), std::ref(obst_config));
            obst.join();
        }
        if(filename != "none")
            dmpc.join();
    }
}

void TrajectoryGenerator::dmpcGenerator(GeneralConfig& gc, YAML::Node& config) {
    auto rate = rclcpp::Rate(1.0/gc.time_step);

    Step s;
    for (const auto& step : config["trajectory"])
    {
        if(!rclcpp::ok())
        {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "DMPC Trajectory step...");
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

            dmpc_publishers[id]->publish(msg);
        }

        rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "DMPC Trajectory finished.");
}



void TrajectoryGenerator::obstGenerator(GeneralConfig& gc, YAML::Node& config) {
    //rate = std::make_shared<rclcpp::Rate>(1.0/gc.time_step);
    auto rate = rclcpp::Rate(1.0/gc.time_step);
    Step s;
    for (const auto& step : config["trajectory"])
    {
        if(!rclcpp::ok())
            return;

        RCLCPP_INFO(this->get_logger(), "OBSTACLE Trajectory step...");
        RCLCPP_INFO(this->get_logger(), "\tID\tx\t\ty\t\tyaw");

        for (const auto& element : step["step"])
        {
            holohover_msgs::msg::HolohoverState msg;

            msg.x   = element["x"].as<double>();
            msg.y   = element["y"].as<double>();
            msg.yaw = element["yaw"].as<double>();
            msg.v_x  = 0.0;
            msg.v_y  = 0.0;
            msg.w_z = 0.0;

            obst_publishers[element["id"].as<int>()]->publish(msg);

            RCLCPP_INFO(this->get_logger(), "\t%d\t%f\t%f\t%f", element["id"].as<int>(), msg.x, msg.y, msg.yaw);
        }

        rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Obstacle Trajectory finished.");
}



GeneralConfig TrajectoryGenerator::parseGeneralConfig(YAML::Node& config)
{
    GeneralConfig gc;

    gc.time_step = config["time_step"].as<double>();
    gc.names     = config["names"].as<std::vector<std::string>>();
    gc.ids       = config["ids"].as<std::vector<int>>();

    if (config["neighbors"])
    {
        for (const auto& neighbor : config["neighbors"]) 
        {
            for (const auto& pair : neighbor)
            {
                int id = pair.first.as<int>();
                gc.neighbors[id] = pair.second.as<std::vector<int>>();
            }
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
