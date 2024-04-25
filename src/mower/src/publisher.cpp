#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/mower_msg.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<interfaces::msg::MowerMsg>("mower_topic", 10);    // CHANGE
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = interfaces::msg::MowerMsg();                      
    message.dist = 1.0;                                        
    message.speed = 2.0;                                        
    message.turntype = this->count_++;                                        
    message.alpha = 3.0;                                        
    message.direction = 2;                                        
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.turntype);    
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces::msg::MowerMsg>::SharedPtr publisher_;        
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}