cmake_minimum_required(VERSION 3.5)
project(gazebo_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(turtle_move src/turtle_move.cpp) 
ament_target_dependencies(turtle_move rclcpp std_msgs turtlesim geometry_msgs)

add_executable(gazebo_move src/gazebo_move.cpp) 
ament_target_dependencies(gazebo_move rclcpp std_msgs geometry_msgs)

add_executable(gazebo_goal src/gazebo_goal.cpp) 
ament_target_dependencies(gazebo_goal rclcpp std_msgs geometry_msgs nav_msgs tf2_geometry_msgs)

install(TARGETS
  turtle_move
  gazebo_move
  gazebo_goal
  DESTINATION lib/${PROJECT_NAME})


ament_package()
