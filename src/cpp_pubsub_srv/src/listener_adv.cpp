#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/location.hpp"
#include "custom_interfaces/srv/authenticate_listener.hpp"
using std::placeholders::_1;

class AdvancedListener : public rclcpp::Node
{
  public:
    AdvancedListener()
    : Node("adv_listener")
    {
      subscription_ = this->create_subscription<custom_interfaces::msg::Location>(
      "custom_topic_adv", 10, std::bind(&AdvancedListener::topic_callback, this, _1));
      publisher_ = this->create_publisher<custom_interfaces::msg::Location>("share_secret", 10);

      client_ = this->create_client<custom_interfaces::srv::AuthenticateListener>("/adv_talker/share_secret_location");

      this->declare_parameter("share_parameter", true);

      timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&AdvancedListener::call_location_service, this));
    }

  private:
    void topic_callback(const custom_interfaces::msg::Location & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: (%f, %f, %f)/%s", 
                      msg.location.x, msg.location.y,
                      msg.location.z, msg.name.c_str());
    }

    void call_location_service()
    {
      if (!client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(), "Service not available...");
        return;
    }

      auto request = std::make_shared<custom_interfaces::srv::AuthenticateListener::Request>();
      request->password = 12345;

      auto future = client_->async_send_request(request,
        std::bind(&AdvancedListener::handle_location_response, this, std::placeholders::_1));

    }

    void handle_location_response(rclcpp::Client<custom_interfaces::srv::AuthenticateListener>::SharedFuture future)
    {
      auto response = future.get();
      
      RCLCPP_INFO(this->get_logger(),
                  "Authenticated location: (%f, %f, %f)/%s",
                  response->secret_location.location.x,
                  response->secret_location.location.y,
                  response->secret_location.location.z,
                  response->secret_location.name.c_str());
      
      const auto & loc = response->secret_location;

      if(this->get_parameter("share_parameter").as_bool())
        publisher_->publish(loc);
    }


  rclcpp::Subscription<custom_interfaces::msg::Location>::SharedPtr subscription_;
  rclcpp::Publisher<custom_interfaces::msg::Location>::SharedPtr publisher_;
  rclcpp::Client<custom_interfaces::srv::AuthenticateListener>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdvancedListener>());
  rclcpp::shutdown();
  return 0;
}