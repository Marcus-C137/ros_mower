#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/mower_msg.hpp"
using std::placeholders::_1;

#include <iostream>
#include <fstream>
#include <cstdint>

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<interfaces::msg::MowerMsg>(
      "mower_topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const interfaces::msg::MowerMsg::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: ");
      RCLCPP_INFO(this->get_logger(), "dist '%f'", msg->dist);
      RCLCPP_INFO(this->get_logger(), "speed: '%f'", msg->speed);
      RCLCPP_INFO(this->get_logger(), "turntype: '%i'", msg->turntype);
      RCLCPP_INFO(this->get_logger(), "alpha: '%f'", msg->alpha);
      RCLCPP_INFO(this->get_logger(), "direction: '%i'", msg->direction);

      // Open serial port (replace "/dev/ttyACM0" with the correct device path)
      std::ofstream serial("/dev/ttyACM0", std::ios::binary);
      
      if (!serial.is_open()) {
          std::cerr << "Error: Failed to open serial port." << std::endl;
          return;
      }

      // Prepare data
      float value1 = 3.14;
      float value2 = 2.718;
      int32_t value3 = 42;
      float value4 = 1.618;
      int32_t value5 = 7;

      // Write data to serial port
      serial.write(reinterpret_cast<const char*>(&value1), sizeof(float));
      serial.write(reinterpret_cast<const char*>(&value2), sizeof(float));
      serial.write(reinterpret_cast<const char*>(&value3), sizeof(int32_t));
      serial.write(reinterpret_cast<const char*>(&value4), sizeof(float));
      serial.write(reinterpret_cast<const char*>(&value5), sizeof(int32_t));

      // Close serial port
      serial.close();

    }

    rclcpp::Subscription<interfaces::msg::MowerMsg>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}