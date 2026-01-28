cmake_minimum_required(VERSION 3.8)
project(teleop_iphone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(arx_ros2 REQUIRED)

# Include directories
include_directories(
  include
)

# Build the main node
add_executable(osc_node
  src/osc_node.cpp
)

ament_target_dependencies(osc_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)

target_link_libraries(osc_node
  pthread
)

# Build the teleop node
add_executable(teleop_iphone
  src/teleop_iphone.cpp
)

ament_target_dependencies(teleop_iphone
  rclcpp
  geometry_msgs
  tf2_ros
  tf2
  arx_ros2
)

# Install targets
install(TARGETS
  osc_node
  teleop_iphone
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
