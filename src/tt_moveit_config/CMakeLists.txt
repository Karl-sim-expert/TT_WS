cmake_minimum_required(VERSION 3.8)
project(tt_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/scene_objects.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
