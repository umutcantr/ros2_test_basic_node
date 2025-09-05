#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// This class subscribes to the "motor_control" topic and reacts to incoming commands
class MotorController : public rclcpp::Node
{
public:
  MotorController()
  : Node("motor_controller")
  {
    // Subscribe to "motor_control" topic with a queue size of 10
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "motor_control", 10, std::bind(&MotorController::topic_callback, this, std::placeholders::_1));
  }

private:
  // This function is called every time a new message arrives on the "motor_control" topic
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Motor çalıştı: '%s'", msg->data.c_str());
    // Here you could add code to actually control the motor hardware
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; // Subscription object
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                           // Initialize ROS 2
  rclcpp::spin(std::make_shared<MotorController>()); // Run the node until shutdown
  rclcpp::shutdown();                                 // Cleanup ROS 2 resources
  return 0;
}
