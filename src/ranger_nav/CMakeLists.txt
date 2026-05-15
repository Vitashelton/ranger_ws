cmake_minimum_required(VERSION 3.8)
project(ranger_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
