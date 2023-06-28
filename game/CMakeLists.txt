cmake_minimum_required(VERSION 3.8)
project(game)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(game src/game.cpp)
add_executable(move src/move_player_turtle.cpp)

ament_target_dependencies(game rclcpp interfaces turtlesim geometry_msgs)
ament_target_dependencies(move rclcpp interfaces)

install(TARGETS
  game
  move
  DESTINATION lib/${PROJECT_NAME})

ament_package()
