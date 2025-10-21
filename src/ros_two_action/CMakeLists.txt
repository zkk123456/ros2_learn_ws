cmake_minimum_required(VERSION 3.8)
project(ros_two_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_two_interface REQUIRED)

# Action Server
add_executable(count_action_server src/count_action_server.cpp)
ament_target_dependencies(count_action_server
  rclcpp
  rclcpp_action
  ros_two_interface
)
# Action Client
add_executable(count_action_client src/count_action_client.cpp)
ament_target_dependencies(count_action_client
  rclcpp
  rclcpp_action
  ros_two_interface
)
install(TARGETS
  count_action_server
  count_action_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
