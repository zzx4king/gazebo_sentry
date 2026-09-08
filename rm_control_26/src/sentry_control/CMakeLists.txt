cmake_minimum_required(VERSION 3.8)
project(sentry_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# 串口通信库 (termios 原生实现, 协议与 test 目录已验证脚本一致)
add_library(serial_port src/serial_port.cpp)
target_include_directories(serial_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(serial_port rclcpp)

# ROS 节点: 订阅 cmd_vel -> 角速度缩放 -> 串口发送
add_executable(control_node src/control_node.cpp)
target_include_directories(control_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(control_node rclcpp geometry_msgs)
target_link_libraries(control_node serial_port)

# 键盘测试节点: 手动发布 cmd_vel, 用于测试串口->MCU 链路
add_executable(teleoptest_node src/teleop_test_node.cpp)
ament_target_dependencies(teleoptest_node rclcpp geometry_msgs)

install(TARGETS control_node teleoptest_node serial_port
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权/风格检查 (非 git 管理的快速开发包)
  set(ament_copyright_FOUND TRUE)
  set(ament_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
