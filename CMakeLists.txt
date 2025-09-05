cmake_minimum_required(VERSION 3.8)
project(motor_control_system)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(command_publisher src/command_publisher.cpp)
ament_target_dependencies(command_publisher rclcpp std_msgs)

add_executable(motor_controller src/motor_controller.cpp)
ament_target_dependencies(motor_controller rclcpp std_msgs)

install(TARGETS
  command_publisher
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
