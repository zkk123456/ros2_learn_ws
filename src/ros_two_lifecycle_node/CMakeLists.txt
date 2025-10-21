cmake_minimum_required(VERSION 3.8)
project(ros_two_lifecycle_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include 
  ${rclcpp_INCLUDE_DIRS}
) 

set(dependencies
  rclcpp 
  std_msgs 
  rclcpp_lifecycle 
  geometry_msgs 
  lifecycle_msgs 
  nav2_util
  nav2_msgs
)

add_executable(lifecycle_node src/my_lifecycle_node.cpp)
ament_target_dependencies(lifecycle_node ${dependencies})
install(TARGETS lifecycle_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
