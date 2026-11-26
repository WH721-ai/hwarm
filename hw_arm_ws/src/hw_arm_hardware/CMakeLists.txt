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


# ── 构建身份标记（2026-11-26 加）
#
# 为什么需要：报告里的日志无法证明"当时跑的这个二进制"就是某个提交构建的。
# 曾经出现过「源码已修、但虚拟机跑的 .so 是旧的」这种无法排除的情况。
# 把提交号和构建时间编进二进制，启动时打印，日志本身就成了证据。
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short=12 HEAD
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  OUTPUT_VARIABLE HW_ARM_GIT_COMMIT
                  OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  execute_process(COMMAND ${GIT_EXECUTABLE} status --porcelain --untracked-files=no
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  OUTPUT_VARIABLE HW_ARM_GIT_DIRTY
                  OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
else()
  set(HW_ARM_GIT_COMMIT "unknown")
  set(HW_ARM_GIT_DIRTY "")
endif()
if(NOT HW_ARM_GIT_COMMIT)
  set(HW_ARM_GIT_COMMIT "unknown")
endif()
if(HW_ARM_GIT_DIRTY)
  set(HW_ARM_GIT_DIRTY_FLAG "+dirty")
else()
  set(HW_ARM_GIT_DIRTY_FLAG "")
endif()
string(TIMESTAMP HW_ARM_BUILD_TIME "%Y-%m-%d %H:%M:%S" UTC)
message(STATUS "hw_arm_hardware build stamp: ${HW_ARM_GIT_COMMIT}${HW_ARM_GIT_DIRTY_FLAG} @ ${HW_ARM_BUILD_TIME}Z")

add_library(hw_arm_hardware SHARED src/hw_arm_hardware.cpp)
target_compile_definitions(hw_arm_hardware PRIVATE
  HW_ARM_GIT_COMMIT="${HW_ARM_GIT_COMMIT}${HW_ARM_GIT_DIRTY_FLAG}"
  HW_ARM_BUILD_TIME="${HW_ARM_BUILD_TIME}")
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
