#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/location.hpp"
#include "custom_interfaces/srv/authenticate_listener.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class AdvancedTalker : public rclcpp::Node
{
  public:
    AdvancedTalker()
    : Node("adv_talker"), password(12345)
    {
      publisher_ = this->create_publisher<custom_interfaces::msg::Location>("custom_topic_adv", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&AdvancedTalker::timer_callback, this));

      location_service_ = this->create_service<custom_interfaces::srv::AuthenticateListener>(
      "/adv_talker/share_secret_location",
      std::bind(&AdvancedTalker::share_location_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    }

  private:
    void timer_callback()
    {
      auto msg = custom_interfaces::msg::Location();
      msg.location.x = 1.0;
      msg.location.y = 1.0;
      msg.location.z = 0.0;
      msg.name = "honolulu";

      publisher_->publish(msg);
    }
    
     void share_location_request(
      const std::shared_ptr<custom_interfaces::srv::AuthenticateListener::Request> request,
      std::shared_ptr<custom_interfaces::srv::AuthenticateListener::Response> response)
    {
      
      if (request->password == this->password) {
        response->secret_location.location.x = 20.0;
        response->secret_location.location.y = 15.0;
        response->secret_location.location.z = 5.0;
        response->secret_location.name = "Haiti";

        RCLCPP_INFO(this->get_logger(), "Authentication successful. Returning location.");
      } else {
        response->secret_location.location.x = 0.0;
        response->secret_location.location.y = 0.0;
        response->secret_location.location.z = 0.0;
        response->secret_location.name = "unauthorized";

        RCLCPP_WARN(this->get_logger(), "Authentication failed. Wrong password: %ld", request->password);
      }
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_interfaces::msg::Location>::SharedPtr publisher_;
    rclcpp::Service<custom_interfaces::srv::AuthenticateListener>::SharedPtr location_service_;
    int password;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdvancedTalker>());
  rclcpp::shutdown();
  return 0;
}