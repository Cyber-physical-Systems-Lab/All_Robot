cmake_minimum_required(VERSION 3.8)
project(cps_robot_description)

find_package(ament_cmake REQUIRED)

# Install the whole model tree so that  package://cps_robot_description/robots/...  resolves.
# usd/ and mjcf/ are excluded: ROS/Gazebo never read them. meshes/flat/ (flattened STLs) is
# kept: gazebo.launch.py uses it for collision meshes Gazebo cannot load from the DAE.
install(DIRECTORY robots
  DESTINATION share/${PROJECT_NAME}
  PATTERN "usd" EXCLUDE
  PATTERN "mjcf" EXCLUDE)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES robots.yaml DESTINATION share/${PROJECT_NAME})

ament_package()
