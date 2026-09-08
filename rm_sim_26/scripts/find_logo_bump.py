#!/usr/bin/env python3
"""定位 logo 凸起：地板层朝上面的 y 层细分 + 凸起块的空间分布。"""
import struct
from collections import defaultdict

import numpy as np

STL = "rm_sim_26/models/rmul_2026/meshes/rmul_2026.stl"
data = open(STL, "rb").read()
n_tri = struct.unpack_from("<I", data, 80)[0]
tris = np.frombuffer(data, dtype=np.uint8, count=n_tri * 50, offset=84).reshape(n_tri, 50)
raw = tris[:, 12:48].copy().view("<f4").reshape(n_tri, 3, 3)  # mm, Y-down
norm = tris[:, :12].copy().view("<f4").reshape(n_tri, 3)
n = norm.astype(np.float64)
n /= np.linalg.norm(n, axis=1, keepdims=True).clip(1e-12)
ny = n[:, 1]

# 世界高度 z_mm = 420 - y_mm（地板 z=0, 低地 z≈12）
z_mm = 420.0 - raw[:, :, 1]

# ---- 1) 朝上面按 z 层细分（0.5mm 步长, z∈[-25, 25]）----
up = ny < -0.9
zc_up = z_mm[up].mean(axis=1)
print("== 朝上水平面 z_mm 直方图（0.5mm 步长，仅非零）==")
h, edges = np.histogram(zc_up, bins=np.arange(-25, 25.5, 0.5))
for c, lo in zip(h, edges[:-1]):
    if c:
        print(f"  z≈{lo:+6.1f}mm : {c}")

# ---- 2) 凸起层（z 在 1~25mm 的朝上面）空间分布 ----
up_idx = np.where(up)[0]
zc_all = z_mm.mean(axis=1)
bump_up = up_idx[(zc_all[up_idx] > 0.8) & (zc_all[up_idx] < 25)]
print(f"\n== 凸起顶面三角形数: {len(bump_up)}")
if len(bump_up):
    p = raw[bump_up].reshape(-1, 3)
    print(f"   原始 x_mm[{p[:,0].min():.0f},{p[:,0].max():.0f}] y_mm[{p[:,1].min():.1f},{p[:,1].max():.1f}] z_mm[{p[:,2].min():.0f},{p[:,2].max():.0f}]")
    # SDF 场地坐标: X = x-6300, Y(场) = z-4300 (mm)
    print(f"   场地 X[{(p[:,0].min()-6300)/1000:+.3f},{(p[:,0].max()-6300)/1000:+.3f}] "
          f"Y[{(p[:,2].min()-4300)/1000:+.3f},{(p[:,2].max()-4300)/1000:+.3f}]")
    for lo, hi in [(0.8, 2.0), (2.0, 4.0), (10.0, 25.0)]:
        sel = bump_up[(zc_all[bump_up] >= lo) & (zc_all[bump_up] < hi)]
        if len(sel):
            pp = raw[sel].reshape(-1, 3)
            print(f"   z∈[{lo},{hi})mm: {len(sel)} tri  场地X[{(pp[:,0].min()-6300)/1000:+.3f},{(pp[:,0].max()-6300)/1000:+.3f}] "
                  f"Y[{(pp[:,2].min()-4300)/1000:+.3f},{(pp[:,2].max()-4300)/1000:+.3f}]")

# ---- 3) 分层聚类：z∈[0.8,4)（几毫米凸起）与 z∈[10,25)（低地）分别做连通块 ----
def clusters(sel_idx):
    q = np.round(raw[sel_idx] / 0.1).astype(np.int64)
    key = q[:, :, 0] * 2**42 + q[:, :, 1] * 2**21 + q[:, :, 2]
    parent = {int(k): int(k) for k in np.unique(key)}
    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for t in range(len(sel_idx)):
        a, b, c = (int(x) for x in key[t])
        ra, rb, rc = find(a), find(b), find(c)
        if ra != rb: parent[rb] = ra
        if ra != rc: parent[rc] = ra
    comp = defaultdict(list)
    for t in range(len(sel_idx)):
        comp[find(int(key[t][0]))].append(t)
    return [sorted(v) for v in comp.values()]

for lo, hi, name in [(0.8, 4.0, "几毫米凸起顶面"), (10.0, 25.0, "低地面")]:
    sel = bump_up[(zc_all[bump_up] >= lo) & (zc_all[bump_up] < hi)]
    print(f"\n== {name} z∈[{lo},{hi})mm: {len(sel)} tri, 连通块:")
    for bi, blk in enumerate(sorted(clusters(sel), key=len, reverse=True)[:14]):
        pp = raw[sel[blk]].reshape(-1, 3)
        print(f"   块{bi}: {len(blk):>3} tri  场地X[{(pp[:,0].min()-6300)/1000:+.3f},{(pp[:,0].max()-6300)/1000:+.3f}] "
              f"Y[{(pp[:,2].min()-4300)/1000:+.3f},{(pp[:,2].max()-4300)/1000:+.3f}] "
              f"z={(420-pp[:,1].mean())/1000:.4f}m")

# ---- 3b) ASCII 分布图（场地 X∈[-6.3,6.3] Y∈[-4.3,4.3], 60x40 格）----
print("\n== 凸起顶面(几毫米层)分布图: '#'=z∈[1,2.5)mm  'o'=低地 z∈[10,25)mm ==")
gx, gy = 60, 40
grid = [[" "]*gx for _ in range(gy)]
sel_all = bump_up
for t in sel_all:
    pp = raw[t].reshape(-1, 3)
    zx = (pp[:, 0].mean()-6300)/1000; zy = (pp[:, 2].mean()-4300)/1000
    ix = int((zx+6.3)/12.6*gx); iy = int((4.3-zy)/8.6*gy)
    if 0 <= ix < gx and 0 <= iy < gy:
        grid[iy][ix] = "#" if zc_all[t] < 5 else "o"
for row in grid:
    print("".join(row))

# ---- 4) 小高度垂直立面（<20mm, 非墙带, 可能是凸起侧面）----
vert = np.abs(ny) < 0.3
c = raw.mean(axis=1)
in_wall = (c[:, 0] < 500) | (c[:, 0] > 12100) | (c[:, 2] < 500) | (c[:, 2] > 8100)
small_vert = vert & ~in_wall & (z_mm.max(axis=1) - z_mm.min(axis=1) < 20)
print(f"\n== 小高度垂直立面（凸起侧面候选）: {small_vert.sum()} tri")
if small_vert.any():
    p = raw[small_vert].reshape(-1, 3)
    print(f"   场地X[{(p[:,0].min()-6300)/1000:+.3f},{(p[:,0].max()-6300)/1000:+.3f}] "
          f"Y[{(p[:,2].min()-4300)/1000:+.3f},{(p[:,2].max()-4300)/1000:+.3f}] "
          f"高度差[{(z_mm[small_vert].max(axis=1)-z_mm[small_vert].min(axis=1)).min():.1f},{(z_mm[small_vert].max(axis=1)-z_mm[small_vert].min(axis=1)).max():.1f}]mm")
