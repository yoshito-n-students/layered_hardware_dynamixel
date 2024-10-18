cmake_minimum_required(VERSION 3.8)
project(layered_hardware_dynamixel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(layered_hardware REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

## Declare a C++ library
add_library(
  layered_hardware_dynamixel_plugins SHARED
  src/layered_hardware_dynamixel_plugins.cpp)
target_include_directories(
  layered_hardware_dynamixel_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(
  layered_hardware_dynamixel_plugins
  dynamixel_workbench_toolbox
  controller_interface
  hardware_interface
  layered_hardware
  pluginlib
  rclcpp
  yaml_cpp_vendor)

## Declare a C++ executable

pluginlib_export_plugin_description_file(layered_hardware plugins.xml)

install(
  TARGETS layered_hardware_dynamixel_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(
  DIRECTORY examples
  DESTINATION share/layered_hardware_dynamixel)

# TODO: declare tests here

ament_package()