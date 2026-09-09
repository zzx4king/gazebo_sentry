from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import math
from tf2_ros import Buffer, TransformListener, TransformException


class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')
        self.declare_parameter('cloud_topic', '/livox/lidar')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_frame', '')
        # costmap 的 global_frame: 用于把 scan 时间戳对齐到 odom→base_link TF 覆盖时刻
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('min_height', 0.12)
        self.declare_parameter('max_height', 1.20)
        self.declare_parameter('range_min', 0.35)
        self.declare_parameter('range_max', 30.0)
        cloud_topic = self.get_parameter('cloud_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(PointCloud2, cloud_topic, self.cb, 10)
        self.pub = self.create_publisher(LaserScan, scan_topic, 10)

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(0.25)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        # 先将斜装雷达点云变换到水平的 base_link，再按高度切片。
        self.z_min = float(self.get_parameter('min_height').value)
        self.z_max = float(self.get_parameter('max_height').value)

        self.n = int((self.angle_max - self.angle_min) / self.angle_increment)

    def cb(self, msg):
        # numpy 向量化处理, 避免逐点 Python 循环 (MID360 ~2万点/帧)
        pts = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)

        frame = self.output_frame or msg.header.frame_id
        if self.output_frame and msg.header.frame_id != self.output_frame:
            try:
                # 慢仿真(RTF<1)下点云时间戳常超前 robot_state_publisher 的 TF,
                # 按点云时刻查 TF 会持续 "extrapolation into the future" 丢帧。
                # 导航时大 yaw 已锁定, livox_lidar -> base_link 时不变, 故取最新可用 TF。
                tf = self.tf_buffer.lookup_transform(
                    self.output_frame, msg.header.frame_id, Time())
            except TransformException as exc:
                self.get_logger().warning(
                    f'等待 {msg.header.frame_id} -> {self.output_frame} TF: {exc}',
                    throttle_duration_sec=2.0)
                return

            q = tf.transform.rotation
            t = tf.transform.translation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            rotation = np.array([
                [1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy - qz*qw),
                 2 * (qx*qz + qy*qw)],
                [2 * (qx*qy + qz*qw), 1 - 2 * (qx*qx + qz*qz),
                 2 * (qy*qz - qx*qw)],
                [2 * (qx*qz - qy*qw), 2 * (qy*qz + qx*qw),
                 1 - 2 * (qx*qx + qy*qy)],
            ], dtype=np.float64)
            pts = pts @ rotation.T
            pts += np.array([t.x, t.y, t.z], dtype=np.float64)

        z = pts[:, 2]
        mask = (z >= self.z_min) & (z <= self.z_max)
        x = pts[mask, 0]
        y = pts[mask, 1]
        if x.size == 0:
            return

        r = np.hypot(x, y)
        mask = (r > self.range_min) & (r < self.range_max)
        if not np.any(mask):
            return
        r = r[mask]
        angle = np.arctan2(y[mask], x[mask])
        idx = ((angle - self.angle_min) / self.angle_increment).astype(np.int64)
        mask = (idx >= 0) & (idx < self.n)
        idx = idx[mask]
        r = r[mask]

        ranges = np.full(self.n, np.inf)
        # 同一扇区取最近点
        np.minimum.at(ranges, idx, r)

        scan = LaserScan()
        # 时间戳对齐(2026-09-09 abort 根因): scan 在点云到达瞬间发布, 而 LIO 的
        # odom→base_link 要等 IMU 覆盖+处理后才发布(同 stamp 晚到 0.1~0.2s)——
        # costmap MessageFilter 收到的 scan 时间戳恒超前 TF 缓存, 查询必失败并积压,
        # 积压超 10s 仿真时间后逐条丢弃; controller 也报 "Transform data too old" abort。
        # 修复: stamp 取 min(点云时刻, odom→base_link TF 最新覆盖时刻),
        # 保证 costmap 对每条 scan 即查即通; 数据新鲜度最多损失 1~2 帧点云周期。
        scan.header = msg.header
        try:
            tr = self.tf_buffer.lookup_transform(
                self.odom_frame, frame, Time())
            if (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) > \
               (tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9):
                scan.header.stamp = tr.header.stamp
        except TransformException:
            # odom→base_link 尚无数据(LIO 未就绪), 丢弃本帧避免下游积压
            return
        scan.header.frame_id = frame
        scan.scan_time = 0.1
        scan.time_increment = 0.0
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = CloudToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
