cmake_minimum_required(VERSION 3.8)
project(tt_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/scene_markers.py scripts/ball_machine.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
