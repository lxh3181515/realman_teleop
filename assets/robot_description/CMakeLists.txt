cmake_minimum_required(VERSION 3.0.2)  # 更新到更现代的版本
project(robot_description)

find_package(catkin REQUIRED COMPONENTS
  xacro  # 支持 XACRO 文件处理
)
catkin_package(
  CATKIN_DEPENDS xacro  # 声明运行时依赖
)


# 安装目录：launch, meshes, urdf
install(DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
