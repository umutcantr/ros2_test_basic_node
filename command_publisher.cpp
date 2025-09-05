#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// This class publishes "run" commands on the "motor_control" topic every second
class CommandPublisher : public rclcpp::Node
{
public:
  CommandPublisher()
  : Node("command_publisher")
  {
    // Create a publisher for std_msgs::msg::String messages on topic "motor_control"
    publisher_ = this->create_publisher<std_msgs::msg::String>("motor_control", 10);
    
    // Create a timer to call timer_callback every 1 second
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CommandPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "run";  // The command to send
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    
    // Publish the message on the topic
    publisher_->publish(message);
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // Publisher object
  rclcpp::TimerBase::SharedPtr timer_;                            // Timer object
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                            // Initialize ROS 2
  rclcpp::spin(std::make_shared<CommandPublisher>()); // Run the node until shutdown
  rclcpp::shutdown();                                  // Cleanup ROS 2 resources
  return 0;
}
