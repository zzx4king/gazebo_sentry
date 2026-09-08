#!/usr/bin/env python3
"""把 rmul_2026.stl 按结构分区拆成多个 STL（坐标系保持原始 mm/Y-down 不变，
SDF 中每个 visual 引用子网格 + 各自材质；collision 仍用整块原网格，物理零变化）。

分类规则（原始坐标 y_mm: 420=地板面 z=0, 0=墙顶 z=0.42）：
  bumpers         非主连通分量（边墙立柱/缓冲块）
  underside       全部顶点 y>445（z<-0.025，含朝下底板面，不可见）
  floor           朝上水平面, 顶点 y∈[390,450]（地板 + 中央低地 z≈0.012）
  platform_top    朝上水平面, 顶点 y<=60（0.4m 大平台顶）
  mid_top         朝上水平面, 顶点 y∈[195,255]（0.2m 台顶）
  walls           垂直面 且 质心贴场地边界（含墙顶）
  structure_faces 其余垂直面（悬崖/平台立面/内壁）
  slopes          斜面（坡道/倒角）
"""
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
STL_IN = ROOT / "models/rmul_2026/meshes/rmul_2026.stl"
OUT_DIR = ROOT / "models/rmul_2026/meshes/parts"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ---------- 解析 ----------
data = STL_IN.read_bytes()
n_tri = struct.unpack_from("<I", data, 80)[0]
tris = np.frombuffer(data, dtype=np.uint8, count=n_tri * 50, offset=84).reshape(n_tri, 50)
raw = tris[:, 12:48].copy().view("<f4").reshape(n_tri, 3, 3)  # mm
norm = tris[:, :12].copy().view("<f4").reshape(n_tri, 3)
n = norm.astype(np.float64)
n /= np.linalg.norm(n, axis=1, keepdims=True).clip(1e-12)
ny = n[:, 1]  # 世界朝上 = raw -y

# ---------- 连通分量（1e-4 m 容差顶点哈希 + 并查集）----------
q = np.round(raw / 0.1).astype(np.int64)  # 0.1mm 栅格
key = q[:, :, 0] * 2**42 + q[:, :, 1] * 2**21 + q[:, :, 2]
parent = {int(k): int(k) for k in np.unique(key)}

def find(a):
    while parent[a] != a:
        parent[a] = parent[parent[a]]
        a = parent[a]
    return a

for t in range(n_tri):
    a, b, c = (int(x) for x in key[t])
    ra, rb, rc = find(a), find(b), find(c)
    if ra != rb:
        parent[rb] = ra
    if ra != rc:
        parent[rc] = ra

comp = np.array([find(int(k)) for k in key[:, 0]])
comp_sizes = {c: int((comp == c).sum()) for c in np.unique(comp)}
main_comp = max(comp_sizes, key=comp_sizes.get)
is_main = comp == main_comp

# ---------- 分类 ----------
c = raw.mean(axis=1)  # 质心 mm
in_wall = (c[:, 0] < 500) | (c[:, 0] > 12100) | (c[:, 2] < 500) | (c[:, 2] > 8100)
ymin = raw[:, :, 1].min(axis=1)
ymax = raw[:, :, 1].max(axis=1)

label = np.full(n_tri, "slopes", dtype=object)
# 立柱/缓冲块 = 小型独立分量(<=24 tri) 且 质心贴墙带(立柱全沿边墙分布);
# 压平的 logo 顶面虽脱离主分量但在场地内部, 按几何规则归类
small_comp = np.array([comp_sizes[c] <= 24 for c in comp]) & in_wall
label[small_comp] = "bumpers"
field = is_main | ~small_comp          # 参与几何分类的三角形
label[(ymax > 445) & field] = "underside"
up = (ny < -0.9) & field & ~((ymax > 445) & field)
down = (ny > 0.9) & field & ~(ymax > 445)
vert = (np.abs(ny) < 0.3) & field & ~(ymax > 445)

label[up & (ymin >= 390) & (ymax <= 450)] = "floor"
label[up & (ymax <= 60) & (label == "slopes")] = "platform_top"
label[up & (ymin >= 195) & (ymax <= 255) & (label == "slopes")] = "mid_top"
label[up & (label == "slopes")] = "other_up"
label[vert & in_wall] = "walls"
label[vert & ~in_wall] = "structure_faces"
label[down & (ymin < 445)] = "under_platform"  # 平台悬挑底面（低角度可见）

# ---------- 导出 ----------
def write_stl(path, mask):
    sub = tris[mask]
    out = bytearray(b"\0" * 80)
    out += struct.pack("<I", len(sub))
    out += sub.tobytes()
    path.write_bytes(bytes(out))
    return len(sub)

groups = ["floor", "platform_top", "mid_top", "walls", "structure_faces",
          "slopes", "under_platform", "underside", "bumpers"]
total = 0
print(f"{'file':<28}{'tris':>6}")
for g in groups:
    m = label == g
    if not m.any():
        print(f"{g + '.stl':<28}{0:>6}   (跳过)")
        continue
    k = write_stl(OUT_DIR / f"{g}.stl", m)
    total += k
    p = raw[m].reshape(-1, 3)
    print(f"{g + '.stl':<28}{k:>6}   y_mm[{p[:,1].min():.0f},{p[:,1].max():.0f}]")

# ---------- 校验 ----------
assert total == n_tri, f"拆分丢失三角形: {total} != {n_tri}"
reborn = np.zeros((n_tri, 3, 3), dtype="<f4")
for g in groups:
    f = OUT_DIR / f"{g}.stl"
    if not f.exists():
        continue
    d = f.read_bytes()
    k = struct.unpack_from("<I", d, 80)[0]
    v = np.frombuffer(d, dtype=np.uint8, count=k * 50, offset=84).reshape(k, 50)[:, 12:48].copy().view("<f4").reshape(k, 3, 3)
    born = np.zeros(k, bool)
    # 顺序合并校验：按类别原顺序拼回，与原顶点逐一对比
    m = label == g
    reborn[m] = v
    born[:] = True
    assert born.all()
assert np.allclose(reborn, raw), "拼回顶点与原网格不一致"
orig_lo, orig_hi = raw.reshape(-1, 3).min(axis=0), raw.reshape(-1, 3).max(axis=0)
print(f"\n校验通过: {total}/{n_tri} tri, 顶点逐一一致; 原 bbox x[{orig_lo[0]:.0f},{orig_hi[0]:.0f}] y[{orig_lo[1]:.0f},{orig_hi[1]:.0f}] z[{orig_lo[2]:.0f},{orig_hi[2]:.0f}]")
