cmake_minimum_required(VERSION 3.16)
project(hw_arm_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenArmCAN REQUIRED)
find_package(hw_arm_algorithms REQUIRED)

add_library(hw_arm_hardware SHARED src/hw_arm_hardware.cpp)
target_include_directories(hw_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hw_arm_hardware PUBLIC
  hw_arm_algorithms::hw_arm_algorithms
  rclcpp::rclcpp
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  std_srvs::std_srvs
  OpenArmCAN::openarm_can)

pluginlib_export_plugin_description_file(hardware_interface hw_arm_hardware.xml)

install(TARGETS hw_arm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES hw_arm_hardware.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(hw_arm_hardware)
ament_export_dependencies(rclcpp hardware_interface pluginlib OpenArmCAN)
ament_package()
