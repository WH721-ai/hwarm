cmake_minimum_required(VERSION 3.16)
project(hw_arm_description)

find_package(ament_cmake REQUIRED)

# meshes/ 是 CAD 导入的 STL（visual/collision 会引用），必须一起安装
install(DIRECTORY urdf srdf config launch rviz docs meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
