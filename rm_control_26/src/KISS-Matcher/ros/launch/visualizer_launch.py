from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    # Jazzy 适配: 原 Humble 版用 os.getcwd() 拼路径, 仅在源码树手动运行时有效;
    # 改为从 install 空间取 share 目录, launch/config/rviz 均随包安装
    pkg_share = get_package_share_directory("kiss_matcher_ros")
    params_file = launch.substitutions.PathJoinSubstitution(
        [pkg_share, "config", "params.yaml"]
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="kiss_matcher_ros",
                executable="registration_visualizer",
                name="registration_visualizer",
                parameters=[params_file],
                output="screen",
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    launch.substitutions.PathJoinSubstitution(
                        [pkg_share, "rviz", "kiss_matcher_reg.rviz"]
                    ),
                ],
                output="screen",
            ),
        ]
    )
