cmake_minimum_required(VERSION 3.8)
project(the_mtt_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(ros2_control REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(robot_state_publisher REQUIRED)


install(
  DIRECTORY
  urdf
  rviz
  launch
  worlds
  src
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Python
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
  launch/yaw_steer.py
  launch/wheel_group_publisher.py
  launch/mtt_controller.py
  # ${PROJECT_NAME}/mtt_controller.py
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
