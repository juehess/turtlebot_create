cmake_minimum_required(VERSION 3.5)
project(create_description)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# install meshes into share/create_description
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# install URDF into share/create_description
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
