cmake_minimum_required(VERSION 3.8)
project(autorccar_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autorccar_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(sim_sensors src/sensors.cpp)
add_executable(sim_run src/simulation.cpp)
add_executable(sim_keyboard src/keyboard.cpp)

ament_target_dependencies(sim_sensors rclcpp std_msgs nav_msgs sensor_msgs autorccar_interfaces)
ament_target_dependencies(sim_run rclcpp std_msgs geometry_msgs)
ament_target_dependencies(sim_keyboard rclcpp std_msgs geometry_msgs)


install(TARGETS
  sim_sensors
  sim_run
  sim_keyboard
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
