#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/location.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Talker : public rclcpp::Node
{
  public:
    Talker()
    : Node("custom_talker"), count_(0)
    {
      publisher_ = this->create_publisher<custom_interfaces::msg::Location>("custom_topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Talker::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto msg = custom_interfaces::msg::Location();
      msg.location.x = 1;
      msg.location.y = 1;
      msg.location.z = 0;
      msg.name = "honolulu";

      RCLCPP_INFO(this->get_logger(), "Publishing: (%f, %f, %f)/%s", 
                    msg.location.x, msg.location.y, 
                    msg.location.z, msg.name.c_str());
      publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_interfaces::msg::Location>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}