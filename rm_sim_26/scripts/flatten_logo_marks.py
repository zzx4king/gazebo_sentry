#!/usr/bin/env python3
"""去掉 rmul_2026 场地三处 logo 毫米级凸起，改为颜色标识贴片 + 矩形边框。

改动（全部在原始 STL 坐标系, mm, Y-down, 地板面 y≈420, 世界 z = 420 - y）：
  1. 三处 logo 凸起的顶面压平到承载面下方 0.5mm（藏于面下, 防破洞/z-fighting）：
       中央低地内图案（z≈13mm → 压到 低地-0.5mm）
       西北角图案（z 2~3mm → 压到 地板-0.5mm）
       东南角图案（同上, 180° 对称）
  2. 凸起的小立面（|nz|<0.3 且高差<20mm）直接删除。
  3. 生成三个同色"贴片+描边框"STL（box 几何, 法向朝外）：
       mark_center.stl 黑（中央图案区 1.4×1.2m 贴片 + 描边框）
       mark_nw.stl     红（西北 1.5×2.0m）
       mark_se.stl     蓝（东南 1.5×2.0m）
  4. 原网格写回 rmul_2026.stl（先备份 .orig），collision 与 visual 同步受益。

用法: python3 scripts/flatten_logo_marks.py   （在 rm_sim_26/ 下执行后需重跑
      split_field_mesh.py 并 colcon build）
"""
import shutil
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "models/rmul_2026/meshes"
STL = MESH_DIR / "rmul_2026.stl"
BAK = MESH_DIR / "rmul_2026.stl.orig"
PARTS = MESH_DIR / "parts"

# 区域盒（SDF 场地坐标 mm, X/Y 以场地中心为原点）：X = raw_x-6300, Y = raw_z-4300
BOX_NW = (-6000, -4500, 2000, 4000)   # xmin xmax ymin ymax
BOX_SE = (4500, 6000, -4000, -2000)
BOX_C = (-700, 700, -600, 600)        # 中央图案区（外扩到贴片尺寸 1.4×1.2m）

FRAME_W = 50       # 描边框条宽 mm
LIFT = 0.5         # 贴片/框相对承载面的抬升 mm；压平面下藏深度同值


def to_raw(x_sdf, y_sdf, z_world):
    """世界/SDF 坐标(mm) -> 原始 STL 坐标(mm): raw=(X+6300, 420-z, Y+4300)"""
    return np.stack([x_sdf + 6300.0, 420.0 - z_world, y_sdf + 4300.0], axis=-1)


def raw_to_z(raw_y):
    return 420.0 - raw_y


def tri_raw_to_field(p):
    """raw 顶点 (...,3)->SDF 场地坐标 (x, y, z_world) mm"""
    return np.stack([p[..., 0] - 6300.0, p[..., 2] - 4300.0, 420.0 - p[..., 1]], axis=-1)


def in_box(field_xy, box):
    return ((field_xy[..., 0] >= box[0]) & (field_xy[..., 0] <= box[1]) &
            (field_xy[..., 1] >= box[2]) & (field_xy[..., 1] <= box[3]))


# ---------------- 读取 ----------------
data = STL.read_bytes()
n_tri = struct.unpack_from("<I", data, 80)[0]
tris = np.frombuffer(data, dtype=np.uint8, count=n_tri * 50, offset=84).reshape(n_tri, 50).copy()
raw = tris[:, 12:48].copy().view("<f4").reshape(n_tri, 3, 3)
norm = tris[:, :12].copy().view("<f4").reshape(n_tri, 3)
n = norm.astype(np.float64)
n /= np.linalg.norm(n, axis=1, keepdims=True).clip(1e-12)
ny = n[:, 1]
z_mm = 420.0 - raw[:, :, 1]            # (n,3) 世界高度 mm
field = tri_raw_to_field(raw)          # (n,3,3) x,y,z_world mm
fc = field.mean(axis=1)                # 质心

# ---------------- 实测承载面高度 ----------------
up = ny < -0.9
zc = z_mm.mean(axis=1)
floor_mask = up & (zc >= -1.0) & (zc < 0.0)     # 主地板层
low_mask = up & (zc >= 11.0) & (zc < 12.45)     # 中央低地面层
assert floor_mask.sum() > 100 and low_mask.sum() > 20, "承载面层判定失败"
z_floor = float(np.median(zc[floor_mask]))
z_low = float(np.median(zc[low_mask]))
print(f"实测承载面: 地板 z={z_floor:+.2f}mm ({floor_mask.sum()} tri), 低地 z={z_low:+.2f}mm ({low_mask.sum()} tri)")

# ---------------- 分类三处凸起 ----------------
vert = np.abs(ny) < 0.3
span = z_mm.max(axis=1) - z_mm.min(axis=1)

def bump_masks(box, z_lo, z_hi):
    inside = in_box(fc[:, :2], box)
    top = up & inside & (zc >= z_lo) & (zc < z_hi)
    side = vert & inside & (span < 20.0)
    return top, side

top_nw, side_nw = bump_masks(BOX_NW, 0.8, 5.0)
top_se, side_se = bump_masks(BOX_SE, 0.8, 5.0)
top_c, side_c = bump_masks(BOX_C, z_low + 0.55, 16.0)
for nm, t, s in [("NW", top_nw, side_nw), ("SE", top_se, side_se), ("C", top_c, side_c)]:
    print(f"区域 {nm}: 顶面 {t.sum()} tri, 立面 {s.sum()} tri")

# ---------------- 压平顶面（顶点级）----------------
flat_z = {"NW": z_floor - LIFT, "SE": z_floor - LIFT, "C": z_low - LIFT}
raw_work = raw.copy()
for name, top in [("NW", top_nw), ("SE", top_se), ("C", top_c)]:
    if not top.any():
        continue
    vt = raw[top].reshape(-1, 1)          # 所有顶点
    y_new = 420.0 - flat_z[name]
    idx = np.where(top)[0]
    raw_work[idx, :, 1] = y_new           # 顶点级: 该三角形全部顶点 y -> y_flat

# ---------------- 删除立面 ----------------
drop = side_nw | side_se | side_c
keep = ~drop
tris_out = tris[keep].copy()
raw_out = raw_work[keep]
# 关键: 把压平后的顶点写回三角形记录（否则写出的仍是原始坐标）
tris_out[:, 12:48] = np.frombuffer(
    raw_out.astype("<f4").tobytes(), dtype=np.uint8).reshape(len(raw_out), 36)
print(f"删除立面 {drop.sum()} tri; 保留 {keep.sum()}/{n_tri}")

# ---------------- 校验 ----------------
zo = 420.0 - raw_out[:, :, 1]
zco = zo.mean(axis=1)
nyo_keep = (np.abs(n[keep][:, 1]) >= 0.3) | True  # 占位
up_out = n[keep][:, 1] < -0.9
# 三区域内不应再存在凸起顶面
for box, zlo, zhi, nm in [(BOX_NW, 0.8, 5.0, "NW"), (BOX_SE, 0.8, 5.0, "SE"), (BOX_C, z_low + 0.55, 16.0, "C")]:
    inside = in_box(fc[keep][:, :2], box)
    bad = up_out & inside & (zco >= zlo) & (zco < zhi)
    assert not bad.any(), f"{nm} 区仍有凸起顶面 {bad.sum()} tri"
# 无零面积三角形
e1 = raw_out[:, 1] - raw_out[:, 0]
e2 = raw_out[:, 2] - raw_out[:, 0]
area2 = np.linalg.norm(np.cross(e1, e2), axis=1)
assert (area2 > 1e-6).all(), f"存在零面积三角形 {(area2 < 1e-6).sum()}"
lo_o, hi_o = raw_out.reshape(-1, 3).min(axis=0), raw_out.reshape(-1, 3).max(axis=0)
lo_i, hi_i = raw.reshape(-1, 3).min(axis=0), raw.reshape(-1, 3).max(axis=0)
assert np.allclose(lo_o, lo_i) and np.allclose(hi_o, hi_i), "bbox 变化!"
print("校验通过: 凸起清零 / 无零面积 / bbox 不变")

# ---------------- 写回主 STL（先备份）----------------
if not BAK.exists():
    shutil.copy2(STL, BAK)
    print(f"已备份原网格 -> {BAK.name}")
out = bytearray(b"\0" * 80)
out += struct.pack("<I", len(tris_out))
out += tris_out.tobytes()
STL.write_bytes(bytes(out))
print(f"写回 {STL.name}: {len(tris_out)} tri")

# ---------------- 复核: 重读写出的文件, 确认凸起顶面在文件里真的清零 ----------------
chk = STL.read_bytes()
n2 = struct.unpack_from("<I", chk, 80)[0]
t2 = np.frombuffer(chk, dtype=np.uint8, count=n2 * 50, offset=84).reshape(n2, 50)
nn = t2[:, :12].copy().view("<f4").reshape(n2, 3).astype(np.float64)
nn /= np.linalg.norm(nn, axis=1, keepdims=True).clip(1e-12)
r2 = t2[:, 12:48].copy().view("<f4").reshape(n2, 3, 3)
zc2 = (420.0 - r2[:, :, 1]).mean(axis=1)
f2 = tri_raw_to_field(r2).mean(axis=1)
up2 = nn[:, 1] < -0.9
for box, zlo, zhi, nm in [(BOX_NW, 0.8, 5.0, "NW"), (BOX_SE, 0.8, 5.0, "SE"), (BOX_C, z_low + 0.55, 16.0, "C")]:
    inside = in_box(f2[:, :2], box)
    bad = up2 & inside & (zc2 >= zlo) & (zc2 < zhi)
    assert not bad.any(), f"写回后 {nm} 区文件内仍有凸起顶面 {bad.sum()} tri"
print("写回复核通过: 文件内凸起顶面已清零")

# ---------------- 生成 mark STL（贴片 + 描边框, box, 法向朝外）----------------
def box_tris(xmin, xmax, ymin, ymax, zmin, zmax):
    """世界坐标 box -> (顶点 raw (12,3,3), 法向 raw (12,3))，从外看逆时针。"""
    xs, ys, zs = [xmin, xmax], [ymin, ymax], [zmin, zmax]
    V = {}
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            for k, z in enumerate(zs):
                V[(i, j, k)] = to_raw(np.float64(x), np.float64(y), np.float64(z))
    faces = [
        ([(0,0,1),(1,0,1),(1,1,1)], [(0,0,1),(1,1,1),(0,1,1)], (0,0,1)),    # +Z
        ([(0,0,0),(0,1,0),(1,1,0)], [(0,0,0),(1,1,0),(1,0,0)], (0,0,-1)),   # -Z
        ([(1,0,0),(1,1,0),(1,1,1)], [(1,0,0),(1,1,1),(1,0,1)], (1,0,0)),    # +X
        ([(0,0,0),(0,0,1),(0,1,1)], [(0,0,0),(0,1,1),(0,1,0)], (-1,0,0)),   # -X
        ([(0,1,0),(0,1,1),(1,1,1)], [(0,1,0),(1,1,1),(1,1,0)], (0,1,0)),    # +Y
        ([(0,0,0),(1,0,0),(1,0,1)], [(0,0,0),(1,0,1),(0,0,1)], (0,-1,0)),   # -Y
    ]
    verts, norms = [], []
    for t1, t2, wn in faces:
        for tri_idx in (t1, t2):
            verts.append([V[i] for i in tri_idx])
            norms.append(wn)
    v = np.array(verts, dtype="<f4")                     # (12,3,3) raw
    w = np.array(norms, dtype=np.float64)                # (12,3) 世界
    nraw = np.stack([w[:, 0], -w[:, 2], w[:, 1]], axis=1)  # 世界->raw 法向
    # 三角形记录: [normal(12B), v1, v2, v3]
    rec = np.zeros((len(v), 50), dtype=np.uint8)
    rec[:, :12] = np.frombuffer(nraw.astype("<f4").tobytes(), dtype=np.uint8).reshape(len(v), 12)
    rec[:, 12:48] = np.frombuffer(v.tobytes(), dtype=np.uint8).reshape(len(v), 36)
    return rec

def ring_boxes(box, z_base, w):
    """区域盒(box)内侧描边: 4 条框条, 外缘=区域边, 内缘内缩 w; 返回 (xmin,xmax,ymin,ymax) 列表"""
    x0, x1, y0, y1 = box
    return [
        (x0, x1, y1 - w, y1),          # 北
        (x0, x1, y0, y0 + w),          # 南
        (x0, x0 + w, y0 + w, y1 - w),  # 西
        (x1 - w, x1, y0 + w, y1 - w),  # 东
    ]

def write_mark(path, box, z_base):
    recs = [box_tris(box[0], box[1], box[2], box[3], z_base, z_base + LIFT)]
    for fb in ring_boxes(box, z_base, FRAME_W):
        recs.append(box_tris(fb[0], fb[1], fb[2], fb[3], z_base + LIFT, z_base + 2 * LIFT))
    rec = np.concatenate(recs)
    out = bytearray(b"\0" * 80)
    out += struct.pack("<I", len(rec))
    out += rec.tobytes()
    path.write_bytes(bytes(out))
    return len(rec)

PARTS.mkdir(parents=True, exist_ok=True)
counts = {
    "mark_center.stl": write_mark(PARTS / "mark_center.stl", BOX_C, z_low),
    "mark_nw.stl": write_mark(PARTS / "mark_nw.stl", BOX_NW, z_floor),
    "mark_se.stl": write_mark(PARTS / "mark_se.stl", BOX_SE, z_floor),
}
for f, c in counts.items():
    print(f"生成 {f}: {c} tri (贴片+4条框)")
print("\n完成。下一步: python3 scripts/split_field_mesh.py && colcon build --packages-select rm_sim_26")
