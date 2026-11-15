cmake_minimum_required(VERSION 3.16)
project(hw_arm_gazebo)

# ─────────────────────────────────────────────────────────────
# 第一阶段：只有「环境 + 组装」（world / launch / 控制器配置）
#   和 hw_arm_mujoco 一样是 install-only 包，colcon build 不需要编译任何东西。
# ─────────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ─────────────────────────────────────────────────────────────
# 算法节点：gravity_node
#   订阅 /joint_states → computeGravity → 发 /hw_arm_effort_controller/commands
#   注意：直接 link hw_arm_algorithms，**不需要** c_api / ctypes
# ─────────────────────────────────────────────────────────────
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hw_arm_algorithms REQUIRED)

add_executable(gravity_node src/gravity_node.cpp)
target_compile_features(gravity_node PRIVATE cxx_std_17)
# 用 plain 签名，才能和 ament_target_dependencies 混用（它内部也是 plain）
target_link_libraries(gravity_node hw_arm_algorithms::hw_arm_algorithms)
ament_target_dependencies(gravity_node
  rclcpp sensor_msgs std_msgs ament_index_cpp)
install(TARGETS gravity_node DESTINATION lib/${PROJECT_NAME})

ament_package()
