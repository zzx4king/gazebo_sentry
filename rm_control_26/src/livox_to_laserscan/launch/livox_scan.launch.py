from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Gazebo directly publishes PointCloud2 on /livox/lidar.
        Node(
            package='livox_to_laserscan',
            executable='pointcloud_to_scan',
            name='cloud_to_scan',
            output='screen',
            parameters=[{
                'cloud_topic': '/livox/lidar',
                'scan_topic': '/scan',
                # 雷达相对车体斜装，必须在水平车体坐标系中生成二维扫描。
                'output_frame': 'base_link',
                # 在 base_link 中切片，去掉地面、车体和高处结构回波。
                'min_height': 0.12,
                'max_height': 1.20,
                'range_min': 0.35,
                'range_max': 30.0,
                'use_sim_time': True,
            }]
        ),

        # base_link -> livox_frame 静态 TF 由 sentry_description 提供, 此处不再发布
    ])
