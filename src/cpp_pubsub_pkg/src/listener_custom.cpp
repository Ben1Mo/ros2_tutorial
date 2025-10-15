#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/location.hpp"
using std::placeholders::_1;

class Listener : public rclcpp::Node
{
  public:
    Listener()
    : Node("custom_listener")
    {
      subscription_ = this->create_subscription<custom_interfaces::msg::Location>(
      "custom_topic", 10, std::bind(&Listener::topic_callback, this, _1));
    }

  private:
    void topic_callback(const custom_interfaces::msg::Location & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: (%f, %f, %f)/%s", 
                      msg.location.x, msg.location.y,
                      msg.location.z, msg.name.c_str());
    }
    rclcpp::Subscription<custom_interfaces::msg::Location>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}