cmake_minimum_required(VERSION 3.5)
project(limo_simulation)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY launch meshes urdf worlds
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(xacro)
ament_export_dependencies(urdf)
ament_package()