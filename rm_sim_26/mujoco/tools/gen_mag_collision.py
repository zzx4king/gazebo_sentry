#!/usr/bin/env python3
"""从 CAD 导出弹仓腔体的碰撞网格.

为什么不用手搓的圆桶: 弹丸容量完全由真实腔体形状决定, 拿 ⌀150 圆桶近似会把
容量算错一个量级。实测中真正兜住弹丸的不只"弹仓"命名的那 15 个零件, 云台防护板、
雷达固定板、NUC 模型、C 板挡板同样在承力 —— 真车上一切结构都是碰撞体。

选零件的判据不用名字(名字不可靠), 而用几何: 零件表面到某发自然堆积弹丸球心的
最小距离小于 3 个弹丸半径, 即视为腔壁(见 wall_parts)。弹丸点云由
tools/measure_mag_capacity.py 产生, 存在 mag_cavity.npz 里。

输出: models/assets/magcol{i}.obj + models/assets/magcol.json (清单)
用法: python3 tools/gen_mag_collision.py
"""
from __future__ import annotations

import json
import os

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
GLB = os.path.join(ROOT, 'models', 'sentry', 'meshes', 'gimbal_link.glb')
OUT = os.path.normpath(os.path.join(HERE, '..', 'models', 'assets'))
CAVITY = os.path.join(HERE, 'mag_cavity.npz')

# SolidWorks Y-up -> ROS Z-up
C = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
R_BALL = 0.0085
MIN_EXTENT = 0.006          # 比这更小的零件(螺钉/垫片)对容量无影响, 只增面数。
                            # 不能设太大: 出弹导轨最长边只有 32mm
FEEDER_POS = np.array([-0.0229, -0.0277, 0.0563])   # 与 gen_mjcf.py 一致
N_SEED = 256                # 导出的实体弹丸候选位置数, 池子按需取前 N 个
FEEDER_REACH = 0.090        # 拨弹盘轴心这个半径内的零件一律算腔壁 (见 wall_parts)
# 转子零件: 它们随 feeder 关节转, 不能当静止腔壁。漏掉这条判据的话, CAD 拨弹盘
# 会以静止几何的身份和 MJCF 里旋转的拨齿重叠, 夹在中间的弹丸第一步就被以
# 3.7 m/s 弹出弹仓 (实测 2/90 发)。
ROTOR_KEYS = ('拨弹盘',)
FEEDER_SWEEP_R = 0.052 + R_BALL     # 拨弹盘扫掠半径 + 弹丸半径
FEEDER_SWEEP_H = 0.026              # 盘面上方拨齿的高度余量


def load_parts(glb: str = GLB, min_extent: float = MIN_EXTENT):
    """按零件拆开 glb, 并转到 ROS 轴系(与 gimbal_link 的可视网格同一坐标系)"""
    sc = trimesh.load(glb, force='scene')
    M = np.eye(4)
    M[:3, :3] = C
    out = []
    for node in sc.graph.nodes_geometry:
        T, gname = sc.graph[node]
        mesh = sc.geometry[gname].copy()
        mesh.apply_transform(T)
        mesh.apply_transform(M)
        if len(mesh.faces) < 4 or mesh.extents.max() < min_extent:
            continue
        out.append((node.split('#')[0], mesh))
    return out


def wall_parts(parts, ball_pts: np.ndarray, reach: float = 3 * R_BALL):
    """真正约束弹丸的零件.

    判据用表面距离而非 AABB: 一块横跨全车的板子, AABB 与弹堆必然重叠, 但它可能
    离弹丸十几厘米。这里取"表面到某发弹丸球心的最小距离 < reach"。reach 给到
    3 个弹丸半径(25.5mm), 比接触判据(1 个半径)宽 —— 静止时没碰到的侧壁, 车一动
    弹丸就会靠上去。

    返回 (选中零件, 弹堆 AABB 下界, 上界)。
    """
    keep = []
    fp = FEEDER_POS.reshape(1, 3)
    for name, mesh in parts:
        if any(k in name for k in ROTOR_KEYS):
            continue                 # 转子, 由 MJCF 的 feeder body 负责
        # 拨弹盘周围的零件一律收进来: 出弹导轨、拨弹挡板、2006 安装座都在这儿,
        # 但测容量时弹丸堆在它们上方够不着, 光靠距离判据会漏掉。漏掉的后果是
        # 拨弹盘把弹丸甩出弹仓 —— 实测 3 秒跑掉 2/90 发, 停转则 0 发。
        if float(mesh.nearest.on_surface(fp)[1][0]) < FEEDER_REACH:
            keep.append((name, mesh))
            continue
        b0, b1 = mesh.bounds
        near = ball_pts[np.all(ball_pts > b0 - reach, axis=1)
                        & np.all(ball_pts < b1 + reach, axis=1)]
        if len(near) == 0:
            continue
        _, dist, _ = mesh.nearest.on_surface(near)
        if dist.min() < reach:
            keep.append((name, mesh))
    return keep, ball_pts.min(axis=0), ball_pts.max(axis=0)


def to_collision(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """转成 MuJoCo 碰撞实际使用的形状 —— 凸包.

    MuJoCo 的 mesh 碰撞本来就只用凸包, 直接导出凸包不改变物理(容量实测就是在
    MuJoCo 的凸包下做的), 但面数从 20 万降到几千, 加载和 broadphase 都快得多。
    """
    try:
        hull = mesh.convex_hull
        if hull.is_volume and hull.volume > 0:
            return hull
    except Exception:
        pass
    return mesh


def main() -> int:
    if not os.path.exists(CAVITY):
        print(f'缺少 {CAVITY} —— 先跑 tools/measure_mag_capacity.py')
        return 1
    z = np.load(CAVITY)
    ball_pts = z['settled']

    parts = load_parts()
    walls, lo, hi = wall_parts(parts, ball_pts)
    os.makedirs(OUT, exist_ok=True)

    manifest, raw_faces = [], 0
    for i, (name, mesh) in enumerate(walls):
        raw_faces += len(mesh.faces)
        col = to_collision(mesh)
        fn = f'magcol{i}.obj'
        col.export(os.path.join(OUT, fn))
        manifest.append({'file': fn, 'part': name, 'faces': int(len(col.faces))})

    # 实体弹丸的初始位置: 从弹堆底部往上取.
    #
    # 不能按"离拨弹盘最近"排序 —— 那样会选进堆顶的点, 而堆顶那些弹丸在满仓
    # (714 发)时是被下面的弹丸托着的; 只放 90 发的话底下是空的, 一开仿真就塌落
    # 160~210mm。按 z 升序取, 得到的是压在仓底、彼此支撑的那几层, 静置位移中位
    # 0.4mm。同层内再按离拨弹盘的水平距离排, 保证供弹口附近先填满。
    r_xy = np.linalg.norm(ball_pts[:, :2] - FEEDER_POS[:2], axis=1)
    # 剔掉落在拨弹盘扫掠体积里的点: 那里第一步就会被拨齿打飞, 不是静态堆放位。
    # 供弹靠 feed() 从弹仓取弹, 不需要预先把弹丸压在盘面上。
    dz = ball_pts[:, 2] - FEEDER_POS[2]
    in_sweep = (r_xy < FEEDER_SWEEP_R) & (dz > -R_BALL) & (dz < FEEDER_SWEEP_H)
    ok = ~in_sweep
    layer = np.round((ball_pts[:, 2] - ball_pts[:, 2].min()) / (2 * R_BALL))
    order = np.lexsort((r_xy[ok], layer[ok]))
    seeds = ball_pts[ok][order][:N_SEED]
    print(f'堆放位: 剔除拨弹盘扫掠区 {int(in_sweep.sum())} 个, 余 {int(ok.sum())} 个')

    with open(os.path.join(OUT, 'magcol.json'), 'w', encoding='utf-8') as f:
        json.dump({'meshes': manifest,
                   'cavity_lo': lo.tolist(), 'cavity_hi': hi.tolist(),
                   'capacity': int(z['capacity']),
                   'seeds': [[round(float(v), 5) for v in p] for p in seeds]},
                  f, ensure_ascii=False, indent=1)

    print(f'腔壁零件 {len(walls)}/{len(parts)}  '
          f'凸包 {sum(m["faces"] for m in manifest)} 面 (原始 {raw_faces})')
    print(f'弹堆包围盒 x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] '
          f"z[{lo[2]:.3f},{hi[2]:.3f}] (已外扩 17mm)")
    print(f'-> {OUT}/magcol*.obj')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
