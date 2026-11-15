cmake_minimum_required(VERSION 3.16)
project(hw_arm_mujoco)

find_package(ament_cmake REQUIRED)

# scripts/   python 脚本（转换器 / viewer 启动器）—— 要可执行
# launch/    display.launch.py（URDF 验证）、mujoco.launch.py
# mjcf/      生成的 MJCF + assets（STL）
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                   GROUP_READ GROUP_EXECUTE
                   WORLD_READ WORLD_EXECUTE
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch mjcf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
