cmake_minimum_required(VERSION 3.5)
project(ros2_test_basic_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(command_publisher command_publisher.cpp)
ament_target_dependencies(command_publisher rclcpp)

add_executable(motor_controller motor_controller.cpp)
ament_target_dependencies(motor_controller rclcpp)

install(TARGETS
  command_publisher
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
