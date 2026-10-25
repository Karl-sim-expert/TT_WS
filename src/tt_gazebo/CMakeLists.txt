cmake_minimum_required(VERSION 3.8)
project(tt_gazebo)

find_package(ament_cmake REQUIRED)

# Ball physics plugin (Gazebo Fortress = ignition-gazebo6)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-math6 REQUIRED)

add_library(BallPhysics SHARED src/BallPhysics.cc)
target_include_directories(BallPhysics PRIVATE include)
target_compile_features(BallPhysics PUBLIC cxx_std_17)
target_link_libraries(BallPhysics PRIVATE
  ignition-plugin1::ignition-plugin1
  ignition-gazebo6::core
  ignition-transport11::core
  ignition-msgs8::core
  ignition-math6::core)

install(TARGETS BallPhysics LIBRARY DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY worlds models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_ball_dynamics test/test_ball_dynamics.cc)
  target_include_directories(test_ball_dynamics PRIVATE include)
  target_compile_features(test_ball_dynamics PUBLIC cxx_std_17)
  add_test(NAME test_ball_dynamics COMMAND test_ball_dynamics)
endif()

ament_package()
