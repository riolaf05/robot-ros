cmake_minimum_required(VERSION 3.8)
project(robot_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Include directories
include_directories(
  include
)

# Arduino Hardware Interface Library
add_library(arduino_hardware_interface SHARED
  robot_ros/hardware_interface/arduino_hardware_interface.cpp
)

target_include_directories(arduino_hardware_interface PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/robot_ros/hardware_interface>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arduino_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware interface plugin
pluginlib_export_plugin_description_file(hardware_interface robot_ros/hardware_interface/arduino_hardware_interface_plugin.xml)

# Install library
install(
  TARGETS arduino_hardware_interface
  DESTINATION lib
)

# Install headers
install(
  DIRECTORY robot_ros/hardware_interface/
  DESTINATION include/robot_ros/hardware_interface
  FILES_MATCHING PATTERN "*.hpp"
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  robot_ros/cmd_to_pwm_driver.py
  robot_ros/GPIO_test.py
  robot_ros/model_test.py
  robot_ros/motor_controller_custom.py
  robot_ros/navbehaviourtree.py
  robot_ros/odom_publisher.py
  robot_ros/publisher_member_function.py
  robot_ros/surveillance_bot.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install other directories
install(DIRECTORY
  config
  description
  maps
  models
  resource
  rviz
  webserver
  worlds
  bt
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(arduino_hardware_interface)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()