cmake_minimum_required(VERSION 3.16)
project(hw_arm_description)

find_package(ament_cmake REQUIRED)

# ──────────────────────────────────────────────────────────────
# 构建期生成一份**展开好的 URDF**，装到 share/urdf/hw_arm.urdf
#
# 为什么需要：ros2_control 的硬件插件（以及 hw_arm_algorithms 的重力补偿）
# 需要一份磁盘上的 URDF 才能建 Pinocchio 模型 —— 运行时 `robot_description`
# 是 launch 里用 xacro 现场展开的字符串，插件拿不到。
#
# ⚠️ 改了 config/*.yaml 或 urdf/*.xacro 都要触发重新生成，
#    所以下面用 GLOB_RECURSE 把整个包设为依赖（踩过坑：只 DEPENDS 主 xacro
#    会导致改了惯量/限位后仍然用旧值）。
# ──────────────────────────────────────────────────────────────
find_program(XACRO_EXECUTABLE xacro)
if(XACRO_EXECUTABLE)
  set(GENERATED_URDF "${CMAKE_CURRENT_BINARY_DIR}/hw_arm.urdf")
  file(GLOB_RECURSE DESC_DEPS CONFIGURE_DEPENDS
       "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro"
       "${CMAKE_CURRENT_SOURCE_DIR}/config/*.yaml")

  add_custom_command(
    OUTPUT  ${GENERATED_URDF}
    COMMAND ${XACRO_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/urdf/hw_arm.urdf.xacro
            use_fake_hardware:=false
            -o ${GENERATED_URDF}
    DEPENDS ${DESC_DEPS}
    COMMENT "展开 hw_arm.urdf.xacro → hw_arm.urdf（重力补偿/其它工具用）")

  add_custom_target(hw_arm_urdf ALL DEPENDS ${GENERATED_URDF})
  install(FILES ${GENERATED_URDF}
          DESTINATION share/${PROJECT_NAME}/urdf)
else()
  message(WARNING "找不到 xacro，跳过生成 hw_arm.urdf")
endif()

# meshes/ 是 CAD 导入的 STL（visual/collision 会引用），必须一起安装
install(DIRECTORY urdf srdf config launch rviz docs meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
