cmake_minimum_required(VERSION 3.8)
project(ros_two_dynamic_parameter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(dynamic_node src/dynamic_parameter.cpp)
ament_target_dependencies(dynamic_node rclcpp)
install(TARGETS dynamic_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
