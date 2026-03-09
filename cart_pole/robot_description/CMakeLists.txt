cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

# Klasörleri share dizinine yükle (RViz ve Gazebo'nun bulabilmesi için)
install(DIRECTORY
  urdf
  meshes
  robot
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
