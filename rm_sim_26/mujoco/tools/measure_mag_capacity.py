#!/usr/bin/env python3
"""实测弹仓自然堆积容量 —— 不估算, 让弹丸自己堆然后数.

为什么不用体积估算: 弹仓包围盒 336x341x237mm = 27 L, 按堆积率折算能得出五千发,
但那是把整个包围盒当成空腔了 —— 里面塞满了 NUC、C 板、云台支撑、雷达固定板。
真实腔体只有其中一小块。凡是靠体积比推出来的数都不可信。

做法(三步, 每步都可独立复查):
  1. 几何: 把云台层所有零件(155 个, 全部当碰撞体 —— 真车就是这样)转到 ROS 轴系,
     在弹仓包围盒里布 18mm 立方格点, 用 trimesh 最近面距离剔掉与结构干涉的点。
     剩下 2787 个候选点, 这里面既有腔内也有车外自由空间, 不做人为区分。
  2. 沉降: 2787 个点全部生成弹丸, 只加重力。车外的自然掉到地上, 腔内的压实。
     数留在弹仓包围盒内的 -> 711 发, 6 秒后不再变化。
  3. 补灌: 从仓顶再倒 784 发。若容量没饱和, 这一步会明显上涨。实测 711 -> 712,
     即已饱和。

注意第 1 步的格点是 18mm 立方排布(密度约 52%), 低于随机密堆(约 62%), 所以
第 2 步之后必须做第 3 步, 否则得到的是下界而不是容量。

输出: tools/mag_cavity.npz  (settled = 仓内弹丸坐标, 供 gen_mag_collision.py 选腔壁)
用法: python3 tools/measure_mag_capacity.py [--grid 0.018] [--topup-layers 4]
耗时: 约 6 分钟 (2787 + 1495 个自由体)
"""
from __future__ import annotations

import argparse
import os
import time

import numpy as np

import mujoco
import trimesh

from gen_mag_collision import R_BALL, load_parts

HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = os.path.join(HERE, '_magwork')
OUTNPZ = os.path.join(HERE, 'mag_cavity.npz')

PROJ_MASS = 0.0032           # 17mm 弹丸, 与 gen_mjcf.py 一致
GEOM = (f'<geom type="sphere" size="{R_BALL}" mass="{PROJ_MASS}" contype="1" '
        'conaffinity="1" friction="0.35 0.005 0.0001" solref="0.006 1" '
        'rgba="0.35 1 0.15 1"/>')


def mag_bbox(parts):
    """弹仓命名零件的包围盒 —— 只用来框定统计范围, 不用来算容量"""
    v = np.vstack([m.vertices for n, m in parts if '弹仓' in n])
    return v.min(axis=0), v.max(axis=0)


def export_parts(parts):
    os.makedirs(SCRATCH, exist_ok=True)
    files = []
    for i, (_, mesh) in enumerate(parts):
        f = os.path.join(SCRATCH, f'p{i}.obj')
        if not os.path.exists(f):
            mesh.export(f)
        files.append(os.path.abspath(f))
    return files


def build_model(files, positions):
    asset = '\n'.join(f'<mesh name="a{i}" file="{f}"/>' for i, f in enumerate(files))
    walls = '\n'.join(
        f'<geom type="mesh" mesh="a{i}" contype="1" conaffinity="1" '
        f'friction="0.6 0.01 0.001" rgba="0.28 0.30 0.34 0.45"/>'
        for i in range(len(files)))
    balls = ''.join(f'<body pos="{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}">'
                    f'<freejoint/>{GEOM}</body>' for p in positions)
    xml = f'''<mujoco model="mag_capacity">
 <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"
         solver="Newton" iterations="20" cone="pyramidal"/>
 <size memory="4000M"/>
 <visual><global offwidth="1600" offheight="1200"/></visual>
 <asset>{asset}</asset>
 <worldbody>
  <light pos="0 0 2" dir="0 0 -1" directional="true"/>
  <geom name="floor" type="plane" size="4 4 .1" pos="0 0 -0.35"
        contype="1" conaffinity="1" rgba=".4 .4 .45 1"/>
  {walls}{balls}
 </worldbody>
</mujoco>'''
    path = os.path.join(SCRATCH, 'capacity.xml')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(xml)
    return mujoco.MjModel.from_xml_path(path)


def _camera(azimuth=115.0, elevation=-10.0):
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.0, -0.03, 0.19]
    cam.distance = 0.85
    cam.azimuth = azimuth
    cam.elevation = elevation
    return cam


def settle(m, n, lo, hi, max_s=14.0, tag='', view=False, video=None):
    """只加重力步进到静止, 返回 (仓内数, 全部坐标).

    view=True 开交互窗口实时看; video=路径 则同时录 mp4。两者都不影响物理 ——
    步进逻辑一致, 只是多了渲染。
    """
    d = mujoco.MjData(m)

    def inside():
        q = d.qpos[:n * 7].reshape(-1, 7)[:, :3]
        return int((np.all(q > lo, axis=1) & np.all(q < hi, axis=1)).sum())

    handle = None
    if view:
        # 注意别写成 import mujoco.viewer —— 那会把 mujoco 绑成本函数的局部名,
        # 于是上面的 mujoco.MjData 直接 UnboundLocalError。
        import mujoco.viewer as mj_viewer
        handle = mj_viewer.launch_passive(m, d, show_left_ui=False,
                                          show_right_ui=False)
        cam = _camera()
        handle.cam.lookat[:] = cam.lookat
        handle.cam.distance, handle.cam.azimuth = cam.distance, cam.azimuth
        handle.cam.elevation = cam.elevation
        print(f'  {tag}窗口已开 —— 关掉窗口会提前结束该阶段')

    frames, renderer = [], None
    if video:
        renderer = mujoco.Renderer(m, 720, 1080)

    every = max(1, int(0.03 / m.opt.timestep))     # 渲染/刷新节流到约 33 Hz
    t0 = time.perf_counter()
    prev = -1
    for i in range(int(max_s / m.opt.timestep)):
        mujoco.mj_step(m, d)
        if handle is not None and i % every == 0:
            if not handle.is_running():
                break
            handle.sync()
        if renderer is not None and i % every == 0:
            renderer.update_scene(d, _camera())
            frames.append(renderer.render())
        if i % 1000 == 999:
            cnt = inside()
            v = np.linalg.norm(d.qvel[:n * 6].reshape(-1, 6)[:, :3], axis=1)
            print(f'  {tag}{d.time:5.1f}s  仓内 {cnt:4d}  线速max {v.max():5.2f} m/s  '
                  f'接触 {d.ncon}')
            if cnt == prev:
                break
            prev = cnt
    print(f'  {tag}耗时 {time.perf_counter() - t0:.0f}s')

    if renderer is not None:
        renderer.close()
        _write_mp4(video, frames, fps=1.0 / (every * m.opt.timestep))
    if handle is not None:
        handle.close()
    return inside(), d.qpos[:n * 7].reshape(-1, 7)[:, :3].copy()


def _write_mp4(path, frames, fps):
    if not frames:
        return
    try:
        import imageio.v2 as imageio
    except ImportError:
        print(f'  (没装 imageio, 跳过 {path}; uv pip install imageio imageio-ffmpeg)')
        return
    imageio.mimwrite(path, frames, fps=fps, macro_block_size=1)
    print(f'  -> {path}  ({len(frames)} 帧 @ {fps:.0f}fps)')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--grid', type=float, default=0.018, help='初始格点间距 m')
    ap.add_argument('--topup-layers', type=int, default=4, help='补灌层数')
    ap.add_argument('--view', action='store_true', help='开交互窗口实时看沉降过程')
    ap.add_argument('--video', metavar='DIR', help='把两个阶段录成 mp4 存到该目录')
    a = ap.parse_args()
    if a.video:
        os.makedirs(a.video, exist_ok=True)

    parts = load_parts()
    print(f'云台层零件 {len(parts)} 个, 共 {sum(len(m.faces) for _, m in parts)} 面 '
          f'(全部作为碰撞体)')
    lo, hi = mag_bbox(parts)
    print(f'弹仓包围盒 x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] '
          f'z[{lo[2]:.3f},{hi[2]:.3f}]  = {np.prod(hi - lo) * 1000:.1f} L (含实体, 非空腔)')

    # ---- 1. 几何: 找不与结构干涉的格点 ----
    big = trimesh.util.concatenate([m for _, m in parts])
    g = [np.arange(lo[k] + R_BALL, hi[k] - R_BALL, a.grid) for k in range(3)]
    P = np.array(np.meshgrid(*g, indexing='ij')).reshape(3, -1).T
    _, dist, _ = big.nearest.on_surface(P)
    seeds = P[dist > R_BALL + 0.0012]
    print(f'候选格点 {len(P)} -> 不干涉 {len(seeds)} ({a.grid * 1000:.0f}mm 间距)')

    files = export_parts(parts)

    # ---- 2. 沉降 ----
    print('沉降中 (车外的会掉到地上, 腔内的压实):')
    m1 = build_model(files, seeds)
    n1, pos1 = settle(m1, len(seeds), lo, hi, tag='', view=a.view,
                      video=os.path.join(a.video, '1_settle.mp4') if a.video else None)
    kept = pos1[np.all(pos1 > lo, axis=1) & np.all(pos1 < hi, axis=1)]
    print(f'沉降后仓内 {n1} 发')

    # ---- 3. 补灌验证饱和 ----
    gx = np.linspace(kept[:, 0].min(), kept[:, 0].max(), 14)
    gy = np.linspace(kept[:, 1].min(), kept[:, 1].max(), 14)
    rng = np.random.default_rng(1)
    extra = [(x + rng.normal(0, .002), y + rng.normal(0, .002),
              hi[2] + 0.03 + 0.021 * layer)
             for layer in range(a.topup_layers) for x in gx for y in gy]
    print(f'从仓顶补灌 {len(extra)} 发:')
    pos = np.vstack([kept, np.array(extra)])
    m2 = build_model(files, pos)
    n2, pos2 = settle(m2, len(pos), lo, hi, tag='', view=a.view,
                      video=os.path.join(a.video, '2_topup.mp4') if a.video else None)

    final = pos2[np.all(pos2 > lo, axis=1) & np.all(pos2 < hi, axis=1)]
    np.savez(OUTNPZ, settled=final, lo=lo, hi=hi,
             capacity=n2, before_topup=n1, grid=a.grid)

    print(f'\n{"=" * 52}')
    print(f'弹仓自然堆积容量: {n2} 发   (补灌前 {n1}, 增量 {n2 - n1})')
    if n2 - n1 > n1 * 0.03:
        print('警告: 补灌后仍在明显上涨, 未饱和 —— 加大 --topup-layers 重测')
    else:
        print('已饱和 (补灌增量 <3%)')
    print(f'-> {OUTNPZ}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
