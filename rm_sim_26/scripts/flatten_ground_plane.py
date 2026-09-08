#!/usr/bin/env python3
"""把 rmul_2026 场地可行驶地面铲平到 z=0：地面层（0<z<25mm）上所有毫米/厘米级
凸起、凹坑、低矮台阶全部压平/清除，保留 200/400mm 平台、围墙等场地结构；
颜色标识贴片重新贴到地板面上。

关键事实（实测）：
  - 地板面（z≈0）在 logo 区、中央岛区、四角墩台下方是挖空的 → 顶面必须"压到 0"
    补成连续地板，不能删除顶面（否则留洞）。
  - 上次压平残留 5 面 3mm 边界立面（logo 区域边线）→ 删除。
  - 中央岛: 48 顶面(12mm) + 32 压平中央 logo 顶面(11.49mm) → 压到 0;
    8 侧面(9→12) + 4 裙边(0→9) + 4 底面(9mm) → 删除。
  - 四角墩台: 8 顶面(20mm) → 压到 0; 32 面口袋墙(20→380mm) 底部下移到 0。
  - NW/SE 凹坑: 112 面(上次压到 -0.505mm) → 抬到 0。
  - 三处颜色贴片 mark_center/nw/se 重新生成, 贴地板面 z=0
    （贴片 [0,0.5]mm, 描边框 [0.5,1.0]mm, 与上一任务同参数, 仅中央从 12mm 改到 0）。

用法: python3 scripts/flatten_ground_plane.py            # 实跑
      python3 scripts/flatten_ground_plane.py --dry-run  # 只打印将处理的面
实跑后需: colcon build --packages-select rm_sim_26
"""
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "models/rmul_2026/meshes"
STL = MESH_DIR / "rmul_2026.stl"
BAK = MESH_DIR / "rmul_2026.stl.pre_flatten"
PARTS = MESH_DIR / "parts"

BOX_NW = (-6000, -4500, 2000, 4000)   # xmin xmax ymin ymax
BOX_SE = (4500, 6000, -4000, -2000)
BOX_C = (-700, 700, -600, 600)
FRAME_W = 50
LIFT = 0.5


def to_raw(x_sdf, y_sdf, z_world):
    """世界/SDF 坐标(mm) -> 原始 STL 坐标(mm): raw=(X+6300, 420-z, Y+4300)"""
    return np.stack([x_sdf + 6300.0, 420.0 - z_world, y_sdf + 4300.0], axis=-1)


# ---------------- 读取 ----------------
data = STL.read_bytes()
n_tri = struct.unpack_from("<I", data, 80)[0]
tris = np.frombuffer(data, dtype=np.uint8, count=n_tri * 50, offset=84).reshape(n_tri, 50).copy()
raw = tris[:, 12:48].copy().view("<f4").reshape(n_tri, 3, 3)
norm = tris[:, :12].copy().view("<f4").reshape(n_tri, 3)
n = norm.astype(np.float64)
n /= np.linalg.norm(n, axis=1, keepdims=True).clip(1e-12)
ny = n[:, 1]
z = 420.0 - raw[:, :, 1]                # (n,3) 世界高度 mm
zc = z.mean(axis=1)
zmin = z.min(axis=1)
zmax = z.max(axis=1)
up = ny < -0.9
down = ny > 0.9
side = np.abs(ny) < 0.3
other = ~(up | down | side)
field = np.stack([raw[..., 0] - 6300.0, raw[..., 2] - 4300.0, z], axis=-1)
fc = field.mean(axis=1)

# 地板参照面（删减前实测）
floor_mask = up & (zc >= -0.1) & (zc < 0.1)
assert floor_mask.sum() > 100, f"地板面判定失败: {floor_mask.sum()} tri"
z_floor = float(np.median(zc[floor_mask]))
print(f"实测地板面 z={z_floor:+.3f}mm ({floor_mask.sum()} tri); 共 {n_tri} tri")

# ---------------- 分类 ----------------
# 压到 z=0 的顶面: 三处 logo(含凹坑) + 中央岛 + 四角墩台
press_up = up & (
    ((zc >= -0.6) & (zc <= -0.4)) |        # NW/SE 凹坑顶面
    ((zc >= 10.0) & (zc <= 13.0)) |        # 中央岛顶 + 中央凹坑
    ((zc >= 19.0) & (zc <= 21.0))          # 四角墩台顶
)
# 删除: 起伏层的非顶面（立面/底面等）, 全部顶点在 -1<z<25 内
del_faces = (zmin > -1.0) & (zmax < 25.0) & ~floor_mask & ~press_up
# 口袋墙（20→380mm 侧面）: 底点下移到 0
pocket = side & (zmin >= 19.0) & (zmin <= 21.0) & (zmax >= 375.0) & (zmax <= 385.0)

print(f"\n压到 z=0 的顶面: {press_up.sum()} tri")
print(f"删除的立面/底面: {del_faces.sum()} tri")
print(f"口袋墙(底部 20→0mm): {pocket.sum()} tri")


def describe(mask, title):
    if not mask.any():
        return
    print(f"\n{title} ({mask.sum()} tri):")
    idx = np.where(mask)[0]
    for i in idx[:80]:
        o = "up" if up[i] else ("down" if down[i] else ("side" if side[i] else "slant"))
        print(f"  #{i}: {o}, z[{zmin[i]:7.2f},{zmax[i]:7.2f}], 质心({fc[i,0]:7.0f},{fc[i,1]:7.0f})")


describe(press_up, "将压平到 z=0")
describe(del_faces, "将删除")
describe(pocket, "将修改(底部下移)")

# ---------------- 落盘 ----------------
if "--dry-run" in sys.argv:
    print("\n[dry-run] 未写盘。确认无误后运行: python3 scripts/flatten_ground_plane.py")
    sys.exit(0)

if not BAK.exists():
    shutil.copy2(STL, BAK)
    print(f"\n已备份当前网格 -> {BAK.name}")

raw_out = raw.copy()
# 1) 顶面压到 z=0 (raw y=420)
if press_up.any():
    idx = np.where(press_up)[0]
    raw_out[idx, :, 1] = 420.0
# 2) 口袋墙底部下移到 0
if pocket.any():
    for i in np.where(pocket)[0]:
        low = z[i] < 21.0
        if low.any():
            raw_out[i, low, 1] = 420.0
# 3) 删除立面/底面
keep = ~del_faces
tris_out = tris[keep].copy()
raw_keep = raw_out[keep]
tris_out[:, 12:48] = np.frombuffer(
    raw_keep.astype("<f4").tobytes(), dtype=np.uint8).reshape(len(raw_keep), 36)

# ---------------- 校验 ----------------
e1 = raw_keep[:, 1] - raw_keep[:, 0]
e2 = raw_keep[:, 2] - raw_keep[:, 0]
area2 = np.linalg.norm(np.cross(e1, e2), axis=1)
assert (area2 > 1e-6).all(), f"存在零面积三角形 {(area2 <= 1e-6).sum()}"
zk = 420.0 - raw_keep[:, :, 1]
zck = zk.mean(axis=1)
zmaxk = zk.max(axis=1)
zmin_k = zk.min(axis=1)
upk = n[keep][:, 1] < -0.9
# 0~200mm 之间不允许再有朝上顶面(除地板/新压平面 zc≈0)
resid = upk & (zck > 0.2) & (zck < 199.0)
assert not resid.any(), f"0~200mm 仍有朝上顶面 {resid.sum()} tri"
# 0~25mm 之间不允许再有非顶面
resid2 = ~upk & (zmaxk < 25.0) & (zmin_k > -1.0)
assert not resid2.any(), f"0~25mm 仍有非顶面 {resid2.sum()} tri"
# 口袋墙底已下移
new_pos = np.cumsum(keep) - 1
for i in np.where(pocket & keep)[0]:
    zv = 420.0 - raw_keep[new_pos[i], :, 1]
    assert zv.min() > -0.01, f"口袋墙底面未下移: #{i} zmin={zv.min():.2f}"

# 压平区域覆盖校验: 各地面特征区域由 z≈0 的朝上面完整覆盖（无洞）
flat_level = upk & (zck >= -0.1) & (zck <= 0.1)
flat_pts = np.stack([raw_keep[flat_level][:, :, 0] - 6300.0,
                     raw_keep[flat_level][:, :, 2] - 4300.0], axis=-1)  # (m,3,2)


def covered(pts, x, y):
    for v in pts:
        a, b, c = v[0], v[1], v[2]
        d1 = (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0])
        d2 = (c[0] - b[0]) * (y - b[1]) - (c[1] - b[1]) * (x - b[0])
        d3 = (a[0] - c[0]) * (y - c[1]) - (a[1] - c[1]) * (x - c[0])
        if (d1 >= -1e-6 and d2 >= -1e-6 and d3 >= -1e-6) or (d1 <= 1e-6 and d2 <= 1e-6 and d3 <= 1e-6):
            return True
    return False


print("\n=== 压平区域地板覆盖校验（50mm 网格） ===")
regions = {
    "中央岛": (-1500, 1500, -1500, 1500),
    "NW logo": (-6000, -4500, 2000, 4000),
    "SE logo": (4500, 6000, -4000, -2000),
    "NW角台": (-6280, -6060, 4060, 4280),
    "NE角台": (6060, 6280, 4060, 4280),
    "SW角台": (-6280, -6060, -4280, -4060),
    "SE角台": (6060, 6280, -4280, -4060),
}
ok_all = True
for name, (x0, x1, y0, y1) in regions.items():
    holes = []
    for x in np.arange(x0 + 25, x1, 50):
        for y in np.arange(y0 + 25, y1, 50):
            if not covered(flat_pts, x, y):
                holes.append((x, y))
    status = "OK" if not holes else f"FAIL: {len(holes)} 个无覆盖点 {holes[:8]}"
    ok_all = ok_all and not holes
    print(f"  {name}: {status}")
assert ok_all, "压平区域仍有洞, 中止"

# ---------------- 写回 ----------------
out = bytearray(b"\0" * 80)
out += struct.pack("<I", len(tris_out))
out += tris_out.tobytes()
STL.write_bytes(bytes(out))
print(f"\n写回 {STL.name}: {len(tris_out)} tri (压平 {press_up.sum()}, 删除 {del_faces.sum()})")

# 写回复核
chk = STL.read_bytes()
n2 = struct.unpack_from("<I", chk, 80)[0]
t2 = np.frombuffer(chk, dtype=np.uint8, count=n2 * 50, offset=84).reshape(n2, 50)
r2 = t2[:, 12:48].copy().view("<f4").reshape(n2, 3, 3)
z2 = 420.0 - r2[:, :, 1]
zc2 = z2.mean(axis=1)
nn = t2[:, :12].copy().view("<f4").reshape(n2, 3).astype(np.float64)
nn /= np.linalg.norm(nn, axis=1, keepdims=True).clip(1e-12)
up2 = nn[:, 1] < -0.9
resid3 = up2 & (zc2 > 0.2) & (zc2 < 199.0)
assert not resid3.any(), "写回后 0~200mm 仍有朝上顶面"
print(f"写回复核通过: {n2} tri, 地面起伏层已清零")

# ---------------- 重生成颜色贴片（全部贴地板面） ----------------
def box_tris(xmin, xmax, ymin, ymax, zmin, zmax):
    xs, ys, zs = [xmin, xmax], [ymin, ymax], [zmin, zmax]
    V = {}
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            for k, z in enumerate(zs):
                V[(i, j, k)] = to_raw(np.float64(x), np.float64(y), np.float64(z))
    faces = [
        ([(0, 0, 1), (1, 0, 1), (1, 1, 1)], [(0, 0, 1), (1, 1, 1), (0, 1, 1)], (0, 0, 1)),
        ([(0, 0, 0), (0, 1, 0), (1, 1, 0)], [(0, 0, 0), (1, 1, 0), (1, 0, 0)], (0, 0, -1)),
        ([(1, 0, 0), (1, 1, 0), (1, 1, 1)], [(1, 0, 0), (1, 1, 1), (1, 0, 1)], (1, 0, 0)),
        ([(0, 0, 0), (0, 0, 1), (0, 1, 1)], [(0, 0, 0), (0, 1, 1), (0, 1, 0)], (-1, 0, 0)),
        ([(0, 1, 0), (0, 1, 1), (1, 1, 1)], [(0, 1, 0), (1, 1, 1), (1, 1, 0)], (0, 1, 0)),
        ([(0, 0, 0), (1, 0, 0), (1, 0, 1)], [(0, 0, 0), (1, 0, 1), (0, 0, 1)], (0, -1, 0)),
    ]
    verts, norms = [], []
    for t1, t2, wn in faces:
        for tri_idx in (t1, t2):
            verts.append([V[i] for i in tri_idx])
            norms.append(wn)
    v = np.array(verts, dtype="<f4")
    w = np.array(norms, dtype=np.float64)
    nraw = np.stack([w[:, 0], -w[:, 2], w[:, 1]], axis=1)
    rec = np.zeros((len(v), 50), dtype=np.uint8)
    rec[:, :12] = np.frombuffer(nraw.astype("<f4").tobytes(), dtype=np.uint8).reshape(len(v), 12)
    rec[:, 12:48] = np.frombuffer(v.tobytes(), dtype=np.uint8).reshape(len(v), 36)
    return rec


def ring_boxes(box, z_base, w):
    x0, x1, y0, y1 = box
    return [
        (x0, x1, y1 - w, y1),
        (x0, x1, y0, y0 + w),
        (x0, x0 + w, y0 + w, y1 - w),
        (x1 - w, x1, y0 + w, y1 - w),
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
    "mark_center.stl": write_mark(PARTS / "mark_center.stl", BOX_C, 0.0),
    "mark_nw.stl": write_mark(PARTS / "mark_nw.stl", BOX_NW, 0.0),
    "mark_se.stl": write_mark(PARTS / "mark_se.stl", BOX_SE, 0.0),
}
for f, c in counts.items():
    print(f"重新生成 {f}: {c} tri (贴片+4条框, 贴地板面 z=0)")

# ---------------- 重跑拆分 ----------------
print("\n重跑 split_field_mesh.py ...")
r = subprocess.run([sys.executable, str(ROOT / "scripts/split_field_mesh.py")],
                   capture_output=True, text=True)
print(r.stdout)
if r.returncode != 0:
    print(r.stderr)
    sys.exit(r.returncode)
print("\n完成。下一步: colcon build --packages-select rm_sim_26")
