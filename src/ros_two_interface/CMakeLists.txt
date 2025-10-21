cmake_minimum_required(VERSION 3.8)
project(ros_two_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
# 设置服务文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
  "action/CountNumbers.action"
  DEPENDENCIES std_msgs
)
# 安装接口文件
install(
  DIRECTORY include/
  DESTINATION include
)
# 安装服务定义
install(
  DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

# 安装动作定义
install(
  DIRECTORY action/
  DESTINATION share/${PROJECT_NAME}/action
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
