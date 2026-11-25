cmake_minimum_required(VERSION 3.16)
project(hw_arm_algorithms VERSION 0.1.0 LANGUAGES CXX)

# ─────────────────────────────────────────────────────────────
# 编译标准与优化
# ─────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Release 下开启优化（动力学计算是热点）
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ─────────────────────────────────────────────────────────────
# 依赖
#   刻意只依赖 Eigen + pinocchio，不引入 rclcpp / hardware_interface。
#   这样本库可以被：实时线程、离线脚本、单元测试、将来的 MCU 代码生成
#   共同复用（见 package.xml 与 docs/DESIGN.md）。
# ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)

# ─────────────────────────────────────────────────────────────
# 库
# ─────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME}
  src/model.cpp      # RobotModel：加载 + 校验
  src/data.cpp       # RobotWorkspace：预分配缓冲
  src/gravity.cpp    # computeGravity
  src/kinematics.cpp # FK / Jacobian / IK / 奇异度
  src/cartesian_trajectory.cpp  # 笛卡尔直线 + RMRC（离线生成路点）
  src/ik_selection.cpp          # IK 多解筛选（硬过滤 + 加权打分）
  src/collision_model.cpp       # 碰撞几何模型（凸包，建一次）
  src/collision_robot.cpp       # 自碰撞检测
  src/collision_world.cpp       # 桌面约束
  src/gravity_identify.cpp      # C-05 静态重力辨识（最小二乘 + 可辨识子空间）
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  yaml-cpp
)

# 库的名字里带命名空间前缀，避免与其它包的 target 冲突
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# hw_arm_hardware 是**共享库**插件，它链接本静态库做重力补偿（C-04），
# 所以这里必须编成位置无关代码，否则链接时报
#   relocation R_X86_64_PC32 ... can not be used when making a shared object
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ─────────────────────────────────────────────────────────────
# 头文件安装
# ─────────────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio)
ament_export_include_directories(include)

# ─────────────────────────────────────────────────────────────
# 测试
# ─────────────────────────────────────────────────────────────
# 微基准（不参与测试，手动运行）
add_executable(bench_gravity test/bench_gravity.cpp)
target_link_libraries(bench_gravity ${PROJECT_NAME})

# 命令行：起点关节角 + 目标位姿 → 笛卡尔直线轨迹的关节路点（YAML）
# 供 goto_xyz.py 调用；也可单独用来离线检查轨迹
add_executable(plan_line_cli tools/plan_line_cli.cpp)
target_link_libraries(plan_line_cli ${PROJECT_NAME})

# 命令行：目标位姿 → IK 解（笛卡尔"点到点"用，只封装 kinematics.cpp）
add_executable(gravity_id_cli tools/gravity_id_cli.cpp)
target_link_libraries(gravity_id_cli ${PROJECT_NAME} yaml-cpp)

add_executable(ik_cli tools/ik_cli.cpp)
target_link_libraries(ik_cli ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 测试需要一个 URDF。默认取同级包的 xacro 并在构建期展开；
  # 也可以通过 -DHW_ARM_URDF_XACRO=<path> 指定。
  set(HW_ARM_URDF_XACRO
      "${CMAKE_CURRENT_SOURCE_DIR}/../hw_arm_description/urdf/hw_arm.urdf.xacro"
      CACHE FILEPATH "hw_arm 的 URDF xacro 路径（测试用）")

  set(HW_ARM_TEST_URDF "${CMAKE_CURRENT_BINARY_DIR}/hw_arm_test.urdf")
  find_program(XACRO_EXECUTABLE xacro)

  if(XACRO_EXECUTABLE AND EXISTS "${HW_ARM_URDF_XACRO}")
    # ⚠️ xacro 会 include 一堆 .xacro 并读取 config/*.yaml。
    # 只 DEPENDS 主 xacro 的话，改了 yaml（关节原点/限位/惯性）不会触发重新生成，
    # 测试就会读到旧的 hw_arm_test.urdf。这里把整个 description 包设为依赖。
    set(HW_ARM_DESC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../hw_arm_description")
    file(GLOB_RECURSE HW_ARM_DESC_DEPS CONFIGURE_DEPENDS
         "${HW_ARM_DESC_DIR}/urdf/*.xacro"
         "${HW_ARM_DESC_DIR}/config/*.yaml")

    add_custom_command(
      OUTPUT  ${HW_ARM_TEST_URDF}
      COMMAND ${XACRO_EXECUTABLE} ${HW_ARM_URDF_XACRO}
              use_fake_hardware:=true
              config_root:=${CMAKE_CURRENT_SOURCE_DIR}/../hw_arm_description/config
              -o ${HW_ARM_TEST_URDF}
      DEPENDS ${HW_ARM_URDF_XACRO} ${HW_ARM_DESC_DEPS}
      COMMENT "展开 hw_arm.urdf.xacro → hw_arm_test.urdf（测试用）")
    add_custom_target(hw_arm_test_urdf ALL DEPENDS ${HW_ARM_TEST_URDF})
    message(STATUS "hw_arm_algorithms: 测试 URDF → ${HW_ARM_TEST_URDF}")
  else()
    message(WARNING
      "hw_arm_algorithms: 找不到 xacro 或 URDF 源；"
      "测试将跳过（可用 -DHW_ARM_URDF_XACRO=<path> 指定）")
  endif()

  # 模型加载与校验
  ament_add_gtest(test_model test/test_model.cpp)
  target_link_libraries(test_model ${PROJECT_NAME})
  target_compile_definitions(test_model PRIVATE
    HW_ARM_DEFAULT_URDF="${HW_ARM_TEST_URDF}")

  # 重力项计算（核心）
  ament_add_gtest(test_gravity test/test_gravity.cpp)
  target_link_libraries(test_gravity ${PROJECT_NAME})
  target_compile_definitions(test_gravity PRIVATE
    HW_ARM_DEFAULT_URDF="${HW_ARM_TEST_URDF}")

  # 运动学：FK / 雅可比 / IK / 奇异度
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${PROJECT_NAME})
  target_compile_definitions(test_kinematics PRIVATE
    HW_ARM_DEFAULT_URDF="${HW_ARM_TEST_URDF}")

  # IK 多解筛选
  ament_add_gtest(test_ik_selection test/test_ik_selection.cpp)
  target_link_libraries(test_ik_selection ${PROJECT_NAME})
  target_compile_definitions(test_ik_selection PRIVATE
    HW_ARM_DEFAULT_URDF="${HW_ARM_TEST_URDF}")



  # C-05 静态重力辨识
  ament_add_gtest(test_gravity_identify test/test_gravity_identify.cpp)
  target_link_libraries(test_gravity_identify ${PROJECT_NAME})
  target_compile_definitions(test_gravity_identify PRIVATE
    HW_ARM_DEFAULT_URDF="${HW_ARM_TEST_URDF}")

  # ik_cli 碰撞检查集成测试（审查报告 C01）：只跑算法 CLI，不接机器人
  add_test(NAME test_ik_cli_collision
           COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ik_cli_collision.sh
                   $<TARGET_FILE:ik_cli> ${HW_ARM_TEST_URDF})
  # 碰撞检测
  ament_add_gtest(test_collision test/test_collision.cpp)
  target_link_libraries(test_collision ${PROJECT_NAME})
  target_compile_definitions(test_collision PRIVATE
    HW_ARM_DEFAULT_URDF="${HW_ARM_TEST_URDF}")

  # 笛卡尔直线轨迹（RMRC）
  ament_add_gtest(test_cartesian_trajectory test/test_cartesian_trajectory.cpp)
  target_link_libraries(test_cartesian_trajectory ${PROJECT_NAME})
  target_compile_definitions(test_cartesian_trajectory PRIVATE
    HW_ARM_DEFAULT_URDF="${HW_ARM_TEST_URDF}")
endif()

ament_package()
