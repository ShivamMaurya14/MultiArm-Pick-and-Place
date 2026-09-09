cmake_minimum_required(VERSION 3.8)
project(ur_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
