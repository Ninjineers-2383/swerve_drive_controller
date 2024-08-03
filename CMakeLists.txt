cmake_minimum_required(VERSION 3.8)
project(swerve_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
)

# find dependencies

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


generate_parameter_library(
  swerve_drive_controller_parameters
  src/swerve_drive_controller_parameters.yaml
)


add_library(swerve_drive_controller SHARED src/swerve_drive_controller.cpp src/swerve_drive_kinematics.cpp src/swerve_drive_odometry.cpp)
target_include_directories(swerve_drive_controller PRIVATE include)

target_link_libraries(
  swerve_drive_controller 
  PUBLIC
  swerve_drive_controller_parameters
)


ament_target_dependencies(
  swerve_drive_controller
  PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
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
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()