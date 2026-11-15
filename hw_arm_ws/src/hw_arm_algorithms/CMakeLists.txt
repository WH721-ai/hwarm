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

# ─────────────────────────────────────────────────────────────
# 库
# ─────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME}
  src/model.cpp      # RobotModel：加载 + 校验
  src/data.cpp       # RobotWorkspace：预分配缓冲
  src/gravity.cpp    # computeGravity
  src/kinematics.cpp # FK / Jacobian / IK / 奇异度
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
)

# 库的名字里带命名空间前缀，避免与其它包的 target 冲突
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

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
endif()

ament_package()
