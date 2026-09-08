#!/usr/bin/env python3
"""全向轮底盘逆运动学: /cmd_vel (Twist) -> 4x 轮速 (Float64).

X 布局四全向轮, 轮位角 theta_i (base 系), 驱动方向为切向:
    w_i = SIGN * (-vx*sin(t) + vy*cos(t) + wz*R) / r

参数来自 CAD 实测: 安装半径 R=0.245 m, 轮半径 r=0.076 m.
SIGN 取决于轮系 z 轴朝向与 gz 关节正方向, 冒烟测试标定 (车动反则取反).
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

WHEEL_RADIUS = 0.076
MOUNT_RADIUS = 0.245
SIGN = -1.0

WHEELS = {
    'wheel_fl': math.radians(45.0),
    'wheel_rl': math.radians(135.0),
    'wheel_rr': math.radians(-135.0),
    'wheel_fr': math.radians(-45.0),
}


class OmniDrive(Node):
    def __init__(self):
        super().__init__('omni_drive')
        self.pubs = {
            name: self.create_publisher(Float64, f'/{name}/cmd_vel', 10)
            for name in WHEELS
        }
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

    def on_cmd(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        for name, theta in WHEELS.items():
            tangential = -vx * math.sin(theta) + vy * math.cos(theta) + wz * MOUNT_RADIUS
            out = Float64()
            out.data = SIGN * tangential / WHEEL_RADIUS
            self.pubs[name].publish(out)


def main():
    rclpy.init()
    node = OmniDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
