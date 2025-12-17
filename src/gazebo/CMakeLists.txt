cmake_minimum_required(VERSION 3.5)
project(gazebo)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY launch meshes
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(xacro)
ament_export_dependencies(urdf)
ament_package()
