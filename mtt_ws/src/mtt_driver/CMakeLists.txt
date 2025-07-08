cmake_minimum_required(VERSION 3.8)
project(mtt_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MttAuxCommand.msg"
  "msg/MttTachometerData.msg"
  DEPENDENCIES std_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS ${PROJECT_NAME}/mtt_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mtt_driver)
install(PROGRAMS ${PROJECT_NAME}/mtt_ros_wrapper.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mtt_ros_wrapper)
install(PROGRAMS ${PROJECT_NAME}/mtt_teleop_joy.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mtt_teleop_joy)
install(PROGRAMS ${PROJECT_NAME}/mtt_test_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mtt_test_node)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/)

ament_package()

