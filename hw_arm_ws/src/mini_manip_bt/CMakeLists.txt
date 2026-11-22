cmake_minimum_required(VERSION 3.16)
project(mini_manip_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

add_executable(bt_executor
  src/bt_executor.cpp
  src/follow_joint_trajectory_node.cpp
)
target_include_directories(bt_executor PRIVATE include)
target_link_libraries(bt_executor
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  rclcpp_action::rclcpp_action
  ${control_msgs_TARGETS}
  behaviortree_cpp::behaviortree_cpp
  behaviortree_ros2::behaviortree_ros2
)

install(TARGETS bt_executor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees launch DESTINATION share/${PROJECT_NAME})

ament_package()
