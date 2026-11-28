cmake_minimum_required(VERSION 3.16)
project(hw_arm_controllers)

if(CMAKE_CXX_STANDARD LESS 17)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hw_arm_algorithms REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/cartesian_impedance_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  Eigen3::Eigen
  hw_arm_algorithms::hw_arm_algorithms
  ament_index_cpp::ament_index_cpp
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

# 插件描述（pluginlib 靠它找到控制器）
pluginlib_export_plugin_description_file(controller_interface hw_arm_controllers_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES hw_arm_controllers_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_plugin_loads test/test_plugin_loads.cpp)
  target_link_libraries(test_plugin_loads ${PROJECT_NAME} pluginlib::pluginlib)
endif()

ament_export_dependencies(controller_interface hardware_interface rclcpp rclcpp_lifecycle
                          realtime_tools hw_arm_algorithms Eigen3)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
