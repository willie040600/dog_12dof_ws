cmake_minimum_required(VERSION 3.8)
project(robot_dog_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config isaac launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/stand_up.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
