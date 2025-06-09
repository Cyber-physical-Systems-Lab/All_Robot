cmake_minimum_required(VERSION 3.5)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf rviz meshes xacro
  DESTINATION share/${PROJECT_NAME}  
)
# If you have a meshes/ directory, also include:
# install(DIRECTORY meshes/
#   DESTINATION share/${PROJECT_NAME}/meshes)

ament_package()

