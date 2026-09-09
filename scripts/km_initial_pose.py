#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KISS-Matcher 初始位姿估计(供 start_navigation.sh 调用)。

流程: 订阅 /fastlio2/world_cloud(odom 系单帧点云)累积(体素去重) ->
双引擎求解 T_map_odom:
  引擎1 KISS-Matcher 全局配准(quatro) -> 体素一致性校验;
  引擎2 几何搜索(墙/箱点 2D FFT 互相关粗搜 + 打包体素键细化) -> 校验。
任一引擎通过校验即用其解; 均失败则退出非零(调用方回退)。
最后与最新 /fastlio2/lio_odom(T_odom_body, 与末帧云同拍发布)复合得
T_map_body, 按 localizer 的 Rz(yaw)·Rx(roll)·Ry(pitch) 约定分解,
stdout 最后一行打印: x y z yaw pitch roll (诊断信息走 stderr)。

注意: world_cloud 每帧仅在 odom 系(非累积), 需多帧拼接; 弱几何场景
(矩形场地+少量箱体)KM 特征匹配可能给出错误解(实测), 体素校验可
靠判别(正确>=95%, 错误<=60%), 故引擎1失败后用确定性几何搜索兜底。
点云不足时可用 --drive 边采集边小范围平移(前后+左右往返)增加视差。

用法:
  python3 km_initial_pose.py --km-bin <run_kiss_matcher路径> --map-pcd <map.pcd> \
      [--collect-sec 12] [--resolution 0.2] [--voxel 0.05] [--drive]
退出码: 0=配准成功且校验通过 2=采集超时 3=KM运行/解析失败 4=内点不足
        5=校验未通过 6=两引擎均未收敛
"""
import argparse
import math
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

VERIFY_VOXEL = 0.25          # 校验体素尺寸
VERIFY_Z_MIN = -0.1          # 校验只统计 z>-0.1 的墙/箱点(地板自相似, 必须排除)


_PCD_TYPES = {
    (b"F", 4): np.float32, (b"F", 8): np.float64,
    (b"I", 1): np.int8, (b"I", 2): np.int16, (b"I", 4): np.int32, (b"I", 8): np.int64,
    (b"U", 1): np.uint8, (b"U", 2): np.uint16, (b"U", 4): np.uint32, (b"U", 8): np.uint64,
}


def load_pcd_xyz(path):
    """读 PCD(支持 ascii/binary, 取 x,y,z 字段)。整体读入再切分, 避免迭代缓冲吞数据。"""
    with open(path, "rb") as f:
        data = f.read()
    pos = 0
    fields, sizes, types, n, fmt = [], [], [], 0, "ascii"
    while True:
        nl = data.find(b"\n", pos)
        if nl < 0:
            break
        line = data[pos:nl]
        pos = nl + 1
        parts = line.split()
        if not parts:
            continue
        if parts[0] == b"FIELDS":
            fields = parts[1:]
        elif parts[0] == b"SIZE":
            sizes = [int(x) for x in parts[1:]]
        elif parts[0] == b"TYPE":
            types = parts[1:]
        elif parts[0] == b"POINTS":
            n = int(parts[1])
        elif parts[0] == b"DATA":
            fmt = parts[1].decode()
            break
    cols = [fields.index(b"x"), fields.index(b"y"), fields.index(b"z")]
    payload = data[pos:]
    if fmt == "ascii":
        pts = np.loadtxt(payload.splitlines(), ndmin=2)
        return pts[:, cols].astype(np.float64)
    # PCD SIZE 为字节数, np.dtype 元组第二项是元素个数, 必须显式映射
    dt = np.dtype([(f"f{i}", _PCD_TYPES[(t, s)])
                   for i, (t, s) in enumerate(zip(types, sizes))])
    arr = np.frombuffer(payload, dtype=dt, count=n)
    return np.column_stack([arr[f"f{c}"] for c in cols]).astype(np.float64)


def pack_keys(pts, vox):
    """体素键打包为 int64(np.isin 向量化校验用)。"""
    k = np.floor(pts / vox).astype(np.int64)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | (k[:, 2] & 0x1FFFFF)


def fast_verify(T, scan_pts, map_wall_keys, z_min=VERIFY_Z_MIN):
    """墙/箱点(排除自相似地板)精确体素命中率。正确对齐实测>=95%, 错误解<=60%。"""
    R, t = T[:3, :3], T[:3, 3]
    p = (R @ scan_pts.T).T + t
    p = p[p[:, 2] > z_min]
    if len(p) < 100:
        return 0.0
    return float(np.isin(pack_keys(p, VERIFY_VOXEL), map_wall_keys).mean())


def floor_z(pts):
    """地板高度 = 最低 10% 点的中位数(地板点通常占 60%+)。"""
    z = pts[:, 2]
    return float(np.median(z[z < np.percentile(z, 10)]))


def geometric_search(scan_pts, map_pts, map_wall_keys):
    """引擎2: 确定性几何搜索。

    墙/箱点(剔地板)投影 2D, 5° 步进 yaw × FFT 互相关粗搜平移,
    top3 候选再做 (yaw ±4°/1°, t ±0.2/0.05) 细化, 用 3D 体素校验选优。
    适用于地面平坦/结构垂直的场地; t_z 取两侧地板高度差。
    返回 (T_map_odom, score) 或 (None, 0)。
    """
    fs, fm = floor_z(scan_pts), floor_z(map_pts)
    s = scan_pts[scan_pts[:, 2] > fs + 0.15, :2]
    m = map_pts[map_pts[:, 2] > fm + 0.15, :2]
    if len(s) < 200 or len(m) < 200:
        return None, 0.0
    cell = 0.25
    lo = np.minimum(s.min(0), m.min(0)) - 2.0
    hi = np.maximum(s.max(0), m.max(0)) + 2.0
    shape = np.ceil((hi - lo) / cell).astype(int) + 1
    L = 1 << int(np.ceil(np.log2(2 * int(shape.max()))))

    def raster(pts2):
        k = np.floor((pts2 - lo) / cell).astype(int)
        k = k[(k[:, 0] >= 0) & (k[:, 0] < shape[0]) &
              (k[:, 1] >= 0) & (k[:, 1] < shape[1])]
        k = np.unique(k, axis=0)
        g = np.zeros((L, L), dtype=np.float32)
        g[k[:, 0], k[:, 1]] = 1.0
        return g

    fa = np.fft.rfft2(raster(m))
    coarse = []
    for deg in range(0, 360, 5):
        th = math.radians(deg)
        c, sn = math.cos(th), math.sin(th)
        B = raster(s @ np.array([[c, sn], [-sn, c]]))
        C = np.fft.irfft2(fa * np.conj(np.fft.rfft2(B)), s=(L, L))
        idx = np.unravel_index(np.argmax(C), C.shape)
        u = idx[0] if idx[0] <= shape[0] - 1 else idx[0] - L
        v = idx[1] if idx[1] <= shape[1] - 1 else idx[1] - L
        coarse.append((float(C[idx]) / max(B.sum(), 1.0), deg, u * cell, v * cell))
    coarse.sort(reverse=True)

    best = (0.0, None)
    for _, deg, tx, ty in coarse[:3]:
        for ddeg in range(-4, 5):
            th = math.radians(deg + ddeg)
            c, sn = math.cos(th), math.sin(th)
            for dx in np.arange(-0.2, 0.21, 0.05):
                for dy in np.arange(-0.2, 0.21, 0.05):
                    T = np.eye(4)
                    T[:3, :3] = [[c, -sn, 0], [sn, c, 0], [0, 0, 1]]
                    T[:3, 3] = [tx + dx, ty + dy, fm - fs]
                    v = fast_verify(T, scan_pts, map_wall_keys)
                    if v > best[0]:
                        best = (v, T)
    return best[1], best[0]


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_zyx(R):
    """R = Rz(yaw)·Rx(roll)·Ry(pitch) 的逆分解(与 localizer relocCB 构造一致)。"""
    roll = math.asin(float(np.clip(R[2, 1], -1.0, 1.0)))
    yaw = math.atan2(-R[0, 1], R[1, 1])
    pitch = math.atan2(-R[2, 0], R[2, 2])
    return yaw, pitch, roll


class Collector(Node):
    def __init__(self, voxel: float):
        super().__init__("km_initial_pose_collector")
        self.voxel = voxel
        self.voxel_map = {}          # 体素键 -> 点, 去重累积
        self.frame_count = 0
        self.first_frame_time = None
        self._latest_odom = None     # (R, t) 最近一帧 lio_odom
        self.last_odom = None        # (R, t) 与末帧云同拍的 odom
        self.cloud_sub = self.create_subscription(
            PointCloud2, "/fastlio2/world_cloud", self.cloud_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/fastlio2/lio_odom", self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        self._latest_odom = (quat_to_rot(p.orientation.x, p.orientation.y,
                                         p.orientation.z, p.orientation.w),
                             np.array([p.position.x, p.position.y, p.position.z]))

    def cloud_cb(self, msg: PointCloud2):
        pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"),
                                       skip_nans=True)
        inv = 1.0 / self.voxel
        for p in pts:
            key = (int(p[0] * inv), int(p[1] * inv), int(p[2] * inv))
            if key not in self.voxel_map:
                self.voxel_map[key] = (float(p[0]), float(p[1]), float(p[2]))
        # lio_node 的 timerCB 先 publishOdometry 再 publishCloud(同一状态),
        # 故云到达时最新 odom 即本帧 odom
        self.last_odom = self._latest_odom
        self.frame_count += 1
        if self.first_frame_time is None:
            self.first_frame_time = time.monotonic()
            print(f"首帧点云到达: {msg.width * msg.height} 点", file=sys.stderr)

    def drive_pattern(self, total_sec: float):
        """前后+左右往返, 净位移≈0, 10Hz 持续发布(omni_drive 指令刷新机制要求)。"""
        phases = [(0.25, 0.0), (0.0, 0.2), (-0.25, 0.0), (0.0, -0.2)]
        vel = Twist()
        per = total_sec / len(phases)
        for vx, vy in phases:
            t_end = time.monotonic() + per
            while time.monotonic() < t_end:
                vel.linear.x, vel.linear.y = float(vx), float(vy)
                self.cmd_pub.publish(vel)
                time.sleep(0.1)
        self.cmd_pub.publish(Twist())

    def save_pcd(self, path: str) -> int:
        pts = list(self.voxel_map.values())
        with open(path, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n")
            f.write(f"COUNT 1 1 1\nWIDTH {len(pts)}\nHEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(pts)}\nDATA ascii\n")
            for p in pts:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
        return len(pts)


def parse_km_output(text: str):
    """解析 run_kiss_matcher 输出: (4x4 矩阵, trans_inliers, succeeded)。"""
    import re
    inliers = None
    m = re.search(r"#\s*trans\s+inliers\s*:\s*(\d+)", text)
    if m:
        inliers = int(m.group(1))
    succeeded = "Registration likely succeeded" in text
    rows = []
    for line in text.splitlines():
        toks = line.split()
        if len(toks) == 4:
            try:
                rows.append([float(t) for t in toks])
            except ValueError:
                pass
    if len(rows) < 4:
        return None, inliers, succeeded
    mat = np.array(rows[-4:])
    if mat.shape != (4, 4) or not np.allclose(mat[3], [0, 0, 0, 1]):
        return None, inliers, succeeded
    return mat, inliers, succeeded


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--km-bin", required=True, help="run_kiss_matcher 可执行文件路径")
    ap.add_argument("--map-pcd", required=True, help="目标点云地图 pcd 路径")
    ap.add_argument("--collect-sec", type=float, default=12.0)
    ap.add_argument("--resolution", type=float, default=0.2)
    ap.add_argument("--voxel", type=float, default=0.05, help="累积体素尺寸(去重)")
    ap.add_argument("--min-inliers", type=int, default=5)
    ap.add_argument("--verify-thresh", type=float, default=0.7,
                    help="墙/箱点体素命中率阈值(正确对齐实测>=0.95, 错误解<=0.6)")
    ap.add_argument("--drive", action="store_true",
                    help="采集期间自动小范围平移增加视差(静止采集不收敛时用)")
    args = ap.parse_args()

    rclpy.init()
    node = Collector(args.voxel)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    if args.drive:
        print("驾驶模式: 采集期间自动平移...", file=sys.stderr)
        driver = threading.Thread(target=node.drive_pattern,
                                   args=(args.collect_sec,), daemon=True)
        driver.start()

    t0 = time.monotonic()
    # 等首帧(最多 30s, LIO 启动需几秒) + 采集 collect_sec
    while rclpy.ok() and node.first_frame_time is None:
        executor.spin_once(timeout_sec=0.2)
        if time.monotonic() - t0 > 30:
            print("错误: 30s 内未收到 /fastlio2/world_cloud, 请确认 LIO 在运行",
                  file=sys.stderr)
            rclpy.shutdown()
            sys.exit(2)
    while rclpy.ok() and time.monotonic() - node.first_frame_time < args.collect_sec:
        executor.spin_once(timeout_sec=0.2)
    if args.drive:
        time.sleep(1.0)  # 等驾驶线程收尾停车
        executor.spin_once(timeout_sec=0.5)

    n_pts = node.save_pcd("/tmp/km_initial_pose_cloud.pcd")
    print(f"采集完成: {node.frame_count} 帧, 去重后 {n_pts} 点", file=sys.stderr)
    if n_pts < 500 or node.last_odom is None:
        print("错误: 点云过少或无里程计", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(2)
    rclpy.shutdown()

    scan_pts = np.array(list(node.voxel_map.values()))
    map_pts = load_pcd_xyz(args.map_pcd)
    map_wall_keys = np.unique(pack_keys(
        map_pts[map_pts[:, 2] > VERIFY_Z_MIN], VERIFY_VOXEL))

    T_solution = None
    # ===== 引擎1: KISS-Matcher 全局配准 =====
    cmd = [args.km_bin, "/tmp/km_initial_pose_cloud.pcd", args.map_pcd,
           str(args.resolution), "nogui", "quatro"]
    print(f"引擎1 KISS-Matcher: {' '.join(cmd)}", file=sys.stderr)
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    except subprocess.TimeoutExpired:
        print("警告: KISS-Matcher 超时, 转几何搜索", file=sys.stderr)
        proc = None
    if proc is not None:
        mat, inliers, succeeded = parse_km_output(proc.stdout)
        print(f"KISS-Matcher: trans_inliers={inliers}, succeeded={succeeded}",
              file=sys.stderr)
        if proc.returncode == 0 and mat is not None and \
                inliers is not None and inliers >= args.min_inliers:
            score = fast_verify(mat, scan_pts, map_wall_keys)
            print(f"引擎1 校验: 墙/箱体素命中率 {score:.1%} "
                  f"(阈值 {args.verify_thresh:.0%})", file=sys.stderr)
            if score >= args.verify_thresh:
                T_solution = mat
        else:
            print("警告: KISS-Matcher 未收敛(内点不足), 转几何搜索", file=sys.stderr)

    # ===== 引擎2: 确定性几何搜索(FFT 互相关) =====
    if T_solution is None:
        print("引擎2 几何搜索: 2D FFT 互相关 + 体素细化...", file=sys.stderr)
        T_geo, geo_score = geometric_search(scan_pts, map_pts, map_wall_keys)
        print(f"引擎2 校验: 墙/箱体素命中率 {geo_score:.1%} "
              f"(阈值 {args.verify_thresh:.0%})", file=sys.stderr)
        if T_geo is not None and geo_score >= args.verify_thresh:
            T_solution = T_geo

    if T_solution is None:
        print("错误: 两引擎均未通过校验, 无法估计初始位姿", file=sys.stderr)
        sys.exit(6)

    # T_map_odom (求解结果, src=odom系云, tgt=map) 复合 T_odom_body(末帧同拍)
    R_mo, t_mo = T_solution[:3, :3], T_solution[:3, 3]
    R_ob, t_ob = node.last_odom
    R_mb = R_mo @ R_ob
    t_mb = R_mo @ t_ob + t_mo
    yaw, pitch, roll = rot_to_zyx(R_mb)
    print(f"T_map_body: t=({t_mb[0]:.3f}, {t_mb[1]:.3f}, {t_mb[2]:.3f}) "
          f"yaw={yaw:.4f} pitch={pitch:.4f} roll={roll:.4f}", file=sys.stderr)
    print(f"{t_mb[0]:.6f} {t_mb[1]:.6f} {t_mb[2]:.6f} "
          f"{yaw:.6f} {pitch:.6f} {roll:.6f}")


if __name__ == "__main__":
    try:
        main()
    except ExternalShutdownException:
        pass
