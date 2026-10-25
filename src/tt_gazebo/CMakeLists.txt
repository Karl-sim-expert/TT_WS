cmake_minimum_required(VERSION 3.8)
project(tt_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
