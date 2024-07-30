cmake_minimum_required(VERSION 3.8)
project(swerve_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


generate_parameter_library(
  swerve_drive_controller_parameters
  src/swerve_drive_controller_parameters.yaml
)

add_library(swerve_drive_controller SHARED src/swerve_drive_controller.cpp)
target_include_directories(swerve_drive_controller PRIVATE include)
target_link_libraries(
  swerve_drive_controller 
  PUBLIC
  swerve_drive_controller_parameters
)
ament_target_dependencies(
  swerve_drive_controller
  PUBLIC
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(controller_interface swerve_drive_controller.xml)



install(
  TARGETS swerve_drive_controller
  DESTINATION lib
)

ament_export_libraries(
  swerve_drive_controller
)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


ament_package()
