import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_share = get_package_share_directory("sentry_nav2_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                package_share, "launch", "navigation2.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "rviz": LaunchConfiguration("rviz"),
            }.items()),
    ])
