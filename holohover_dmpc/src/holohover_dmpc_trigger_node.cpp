#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "control_dmpc_settings.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class HolohoverDmpcTrigger : public rclcpp::Node
{
  public:
    HolohoverDmpcTrigger()
    : Node("holohover_dmpc_trigger"), control_settings(load_control_dmpc_settings(*this)), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::UInt64>("/dmpc/trigger", 10);
      std::this_thread::sleep_for(std::chrono::seconds(30));
      RCLCPP_INFO(get_logger(), " - - - STARTING TRIGGERING NOW - - - ");
      timer_ = this->create_wall_timer(
      std::chrono::duration<double>(control_settings.dmpc_period), std::bind(&HolohoverDmpcTrigger::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::UInt64();
      count_++;
      message.data = count_;
      publisher_->publish(message);
    }
    ControlDMPCSettings control_settings;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HolohoverDmpcTrigger>());
  rclcpp::shutdown();
  return 0;
}