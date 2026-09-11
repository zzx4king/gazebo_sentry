cmake_minimum_required(VERSION 3.8)
project(hero_mpc_controller)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# 查找依赖
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# =============================================================================
# acados 配置
# =============================================================================
# 设置 acados 安装路径（请根据实际安装位置修改）
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(ACADOS_INSTALL_DIR "${PROJECT_ROOT}/src/third_party/acados" CACHE PATH "Path to acados installation")

# acados 生成代码路径
set(ACADOS_GENERATED_CODE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/model/c_generated_code")

# 检查 acados 是否存在
if(NOT EXISTS "${ACADOS_INSTALL_DIR}/include/acados_c")
  message(WARNING "acados not found at ${ACADOS_INSTALL_DIR}. Please set ACADOS_INSTALL_DIR.")
endif()

# acados 头文件路径
set(ACADOS_INCLUDE_DIRS
  ${ACADOS_INSTALL_DIR}/include
  ${ACADOS_INSTALL_DIR}/include/acados
  ${ACADOS_INSTALL_DIR}/include/blasfeo/include
  ${ACADOS_INSTALL_DIR}/include/hpipm/include
  ${ACADOS_GENERATED_CODE_DIR}
)

# acados 库路径
set(ACADOS_LIB_DIR ${ACADOS_INSTALL_DIR}/lib)

# acados 库
set(ACADOS_LIBS
  ${ACADOS_LIB_DIR}/libacados.so
  ${ACADOS_LIB_DIR}/libhpipm.so
  ${ACADOS_LIB_DIR}/libblasfeo.so
)

# 收集 acados 生成的源文件（排除包含 main 函数的测试文件）
file(GLOB_RECURSE ACADOS_GENERATED_SOURCES
  ${ACADOS_GENERATED_CODE_DIR}/*.c
)
# 排除 main_*.c 文件（这些是 acados 生成的测试程序，包含 main 函数）
list(FILTER ACADOS_GENERATED_SOURCES EXCLUDE REGEX "main_.*\\.c$")

# =============================================================================
# 构建共享库
# =============================================================================
add_library(${PROJECT_NAME} SHARED
  src/mpc_wrapper.cpp
  src/hero_mpc_controller.cpp
  ${ACADOS_GENERATED_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ACADOS_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${ACADOS_LIBS}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    # libacados.so itself depends on qpOASES/HPIPM/BLASFEO.  DT_RUNPATH is
    # not used to resolve those transitive dependencies, so keep the bundled
    # runtime beside the controller and use a legacy DT_RPATH rooted at the
    # installed plugin directory.
    INSTALL_RPATH "$ORIGIN"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)
target_link_options(${PROJECT_NAME} PRIVATE "LINKER:--disable-new-dtags")

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  interfaces
)

# 定义构建库时的宏
target_compile_definitions(${PROJECT_NAME} PRIVATE HERO_MPC_CONTROLLER_BUILDING_LIBRARY)

# =============================================================================
# 安装
# =============================================================================
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Bundle the acados runtime required by the generated MPC solver.  This keeps
# the installed Nav2 plugin self-contained instead of depending on a source-tree
# path or a manually configured LD_LIBRARY_PATH.
install(FILES
  ${ACADOS_LIB_DIR}/libacados.so
  ${ACADOS_LIB_DIR}/libhpipm.so
  ${ACADOS_LIB_DIR}/libblasfeo.so
  ${ACADOS_LIB_DIR}/libblasfeo.so.0
  ${ACADOS_LIB_DIR}/libqpOASES_e.so
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# =============================================================================
# 注册 Nav2 插件
# =============================================================================
pluginlib_export_plugin_description_file(nav2_core hero_mpc_controller.xml)

# =============================================================================
# 导出
# =============================================================================
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  interfaces
)

# =============================================================================
# 测试
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
