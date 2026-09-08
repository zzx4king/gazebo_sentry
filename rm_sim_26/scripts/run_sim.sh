#!/bin/bash
cd /home/robomaster/project/rm_gazebo_26/rm_sim_26

# 0. 清理上次运行残留的 Gazebo / ROS 进程（残留服务端会导致新启动的 world 加载不到 GUI）
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "rmuc_2025_sim" 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f omni_drive 2>/dev/null
sleep 1

# 1. 刷新 ROS 2 工作空间
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 2. 启动仿真
ros2 launch rm_sim_26 rmuc_2025_sim.launch.py
