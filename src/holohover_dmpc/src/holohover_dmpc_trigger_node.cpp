#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

using namespace std::chrono_literals;

class HolohoverDmpcTrigger : public rclcpp::Node
{
  public:
    HolohoverDmpcTrigger() : Node("holohover_dmpc_trigger")
    {
      auto publisher_ = this->create_publisher<std_msgs::msg::UInt64>("/dmpc/trigger", 10);
      std::this_thread::sleep_for(std::chrono::seconds(15));
      RCLCPP_INFO(get_logger(), " - - - STARTING TRIGGERING NOW - - - ");
      auto message = std_msgs::msg::UInt64();
      message.data = 0;
      publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HolohoverDmpcTrigger>());
  rclcpp::shutdown();
  return 0;
}