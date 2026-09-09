import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MAP = "/home/robomaster/project/rm_gazebo_26/rm_control_26/maps/world1/pgm/map.yaml"

def generate_launch_description():
    package_share = get_package_share_directory("sentry_nav2_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map", default_value=DEFAULT_MAP),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(package_share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="nav2_map_server", executable="map_server",
            name="map_server", output="screen",
            parameters=[params_file, {
                "yaml_filename": map_yaml, "use_sim_time": use_sim_time}]),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_map", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time, "autostart": True,
                "node_names": ["map_server"]}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav2_share, "launch", "navigation_launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": "true",
            }.items()),
        Node(
            package="rviz2", executable="rviz2", name="nav2_rviz",
            output="screen", condition=IfCondition(use_rviz),
            arguments=["-d", os.path.join(
                package_share, "rviz", "nav2_red_scan.rviz")],
            parameters=[{"use_sim_time": use_sim_time}]),
    ])
