cmake_minimum_required(VERSION 3.8)
project(sentry_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz behavior_trees
  DESTINATION share/sentry_nav2_bringup
)

ament_package()
