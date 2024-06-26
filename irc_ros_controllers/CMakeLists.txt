cmake_minimum_required(VERSION 3.5)
project(irc_ros_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(irc_ros_hardware REQUIRED)
find_package(irc_ros_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(INCLUDE_DEPENDS
  ament_cmake
  controller_interface
  example_interfaces
  hardware_interface
  irc_ros_hardware
  irc_ros_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  rcpputils
  realtime_tools
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/dio_controller.cpp
  src/platform_controller.cpp
  src/Velocity_PositionController.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}
  ${INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface platform_controller.xml)
pluginlib_export_plugin_description_file(controller_interface dio_controller.xml)
pluginlib_export_plugin_description_file(controller_interface Velocity_PositionController.xml)


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


ament_export_dependencies(
  ${INCLUDE_DEPENDS}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()