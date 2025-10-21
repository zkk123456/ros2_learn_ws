cmake_minimum_required(VERSION 3.8)
project(package_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hello_world_publisher src/hello_world_publisher.cpp)
ament_target_dependencies(hello_world_publisher rclcpp std_msgs)

# 编译动态库
set(library_name ${PROJECT_NAME}_core)
add_library(${library_name} SHARED src/hello_world_publisher.cpp)   # 添加头文件地址
#有将头文件共享出去
target_include_directories(${library_name} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )
# 用于代替传统的target_link_libraries
ament_target_dependencies(${library_name}
        PUBLIC
        rclcpp
        std_msgs
)

# 为了使下游文件可以访问
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)# 注册 导出头文件
install(
        DIRECTORY include/
        DESTINATION include)# 注册 导出库文件
install(
        TARGETS ${library_name}# 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT ${library_name}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

# 安装可执行文件
install(TARGETS
  hello_world_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
