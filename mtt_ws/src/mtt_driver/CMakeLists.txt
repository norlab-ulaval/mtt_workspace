cmake_minimum_required(VERSION 3.8)
project(mtt_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MttAuxCommand.msg"
  DEPENDENCIES std_msgs
)


ament_python_install_package(${PROJECT_NAME})

install(TARGETS 
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/mtt_driver.py
  ${PROJECT_NAME}/mtt_ros_wrapper.py
  ${PROJECT_NAME}/mtt_teleop_joy.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

