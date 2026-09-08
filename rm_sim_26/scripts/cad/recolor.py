#!/usr/bin/env python3
"""给 GLB 上色: 结构件按 CAD 材料密度分类, 外购件/裁判系统按实物本色覆盖.

Onshape 导出的外观全是白/浅灰(SolidWorks 外观未随文件传递), 与实车不符。
两层规则:
  1) OVERRIDES —— 按零件名匹配, 优先级最高。裁判系统模块、相机、雷达、
     全向轮辊子等有确定实物颜色, 不该刷黑。
  2) 密度分类 —— 其余结构件用 densities.json(质量/体积反推)判材料,
     玻纤板与打印件哑光黑, 铝件钢件保持金属本色。
另外: 原始外观里高饱和度的零件(镜头/灯条)保留其色相, 不做覆盖。
"""
import json
import os
import re

import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

SRC, DST = 'meshes_slim', 'meshes_black'
os.makedirs(DST, exist_ok=True)

# ---- 按名字覆盖: (关键词, 排除正则, sRGB, metallic, roughness, alpha, 说明) ----
# alpha < 1 的会设为 alphaMode='BLEND' + doubleSided, 才能真正透明
OVERRIDES = [
    (r'滚子1',        None,          (0.88, 0.88, 0.86), 0.02, 0.45, 1.00, '全向轮辊子 白色聚氨酯'),
    (r'MV-CA016',     None,          (0.62, 0.63, 0.65), 0.72, 0.30, 1.00, '海康工业相机 银灰铝'),
    (r'相机模型',      None,          (0.62, 0.63, 0.65), 0.72, 0.30, 1.00, '相机'),
    (r'测速模块',      r'范围',        (0.82, 0.83, 0.84), 0.02, 0.45, 1.00, '裁判系统测速模块 白塑料'),
    (r'装甲模块',      None,          (0.80, 0.81, 0.83), 0.03, 0.45, 1.00, '裁判系统装甲模块'),
    (r'灯条模块',      None,          (0.86, 0.87, 0.89), 0.02, 0.32, 1.00, '裁判系统灯条'),
    (r'场地交互模块',   r'安装|定位',   (0.80, 0.81, 0.83), 0.03, 0.45, 1.00, '裁判系统场地交互'),
    (r'主控模块',      r'安装|支架',   (0.80, 0.81, 0.83), 0.03, 0.45, 1.00, '裁判系统主控'),
    (r'电源管理',      r'固定',        (0.30, 0.31, 0.32), 0.10, 0.55, 1.00, '电源管理模块'),
    (r'定位模块',      r'固定|安装|板', (0.80, 0.81, 0.83), 0.03, 0.45, 1.00, '裁判系统定位模块'),
    # Mid360 机身: 实物偏深灰, 这里刻意提亮成浅银灰 —— 仿真里要一眼看出传感器
    # 位置与朝向, 可辨识优先于照片级还原。想还原实物改成 (0.30,0.31,0.33) 即可。
    # 镜头是深蓝(17,1,151), 由饱和度规则自动保留, 不受此条影响。
    (r'MID-360',      None,          (0.70, 0.72, 0.75), 0.55, 0.30, 1.00, 'Livox Mid360 机身'),
    (r'荧光充能',      None,          (0.72, 0.88, 0.38), 0.02, 0.40, 1.00, '荧光标记'),
    # 亚克力件由密度识别(见 ACRYLIC_*), 不在此处按名字匹配
    (r'C_MAIN_BOARD', None,          (0.10, 0.28, 0.14), 0.10, 0.55, 1.00, 'C板 PCB 墨绿'),
]

# ---- 密度分类调色板 (sRGB) ----
PALETTE_SRGB = {
    '碳纤维': ((0.105, 0.110, 0.118), 0.10, 0.42),
    '玻纤':   ((0.130, 0.137, 0.143), 0.06, 0.62),
    '打印':   ((0.170, 0.172, 0.180), 0.03, 0.82),
    '铝':     ((0.620, 0.632, 0.648), 0.90, 0.30),   # 金属本色, 不压暗
    '钢':     ((0.540, 0.552, 0.570), 0.92, 0.24),
    '铜':     ((0.560, 0.420, 0.260), 0.90, 0.35),
    '默认':   ((0.140, 0.145, 0.153), 0.20, 0.60),
}
BANDS = [('碳纤维', 1500, 1700), ('玻纤', 1800, 2100), ('打印', 900, 1400),
         ('铝', 2600, 2850), ('钢', 7600, 8100), ('铜', 8300, 9000)]

SAT_KEEP = 60          # 原始饱和度超过此值 -> 保留色相(镜头/灯条)

# 亚克力(PMMA) ρ≈1160~1190, 正好落在打印件档(900~1400)里, 只按密度会被刷黑。
# 用 密度 + 名字含防护类词 双条件识别, 单独给透明材质。
ACRYLIC_RHO = (1150, 1250)
ACRYLIC_NAME = r'防护|保护|罩|挡板|亚克力|窗'
ACRYLIC_MAT = ((0.86, 0.90, 0.92), 0.00, 0.05, 0.18)   # sRGB, metallic, roughness, alpha


def srgb_to_linear(c):
    c = np.asarray(c, dtype=float)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def material_of(rho):
    for name, lo, hi in BANDS:
        if lo <= rho <= hi:
            return name
    return '默认'


rows = json.load(open('densities.json'))
dens = {n: r for n, m, v, r in rows}
keys = sorted(dens, key=len, reverse=True)


def lookup_density(node_name):
    base = node_name.split('#')[0].split(' <')[0].split('__')[0]
    for cand in (base, base.rsplit('-', 1)[0]):
        if cand in dens:
            return dens[cand]
    for k in keys:
        if len(k) >= 8 and base.startswith(k):
            return dens[k]
    return None


def override_of(node_name):
    for pat, excl, rgb, met, rough, alpha, note in OVERRIDES:
        if re.search(pat, node_name) and not (excl and re.search(excl, node_name)):
            return (rgb, met, rough, alpha), note
    return None, None


def orig_color(geo):
    mat = getattr(geo.visual, 'material', None)
    if mat is None:
        return None
    for attr in ('baseColorFactor', 'main_color'):
        c = getattr(mat, attr, None)
        if c is not None:
            a = np.array(c, dtype=float)[:3]
            return a if a.max() > 1.5 else a * 255
    return None


def make(rgb, met, rough, name, alpha=1.0):
    kw = {}
    if alpha < 1.0:
        # glTF 透明必须显式声明, 否则渲染器按不透明处理
        kw = dict(alphaMode='BLEND', doubleSided=True)
    return PBRMaterial(baseColorFactor=np.concatenate([srgb_to_linear(rgb), [alpha]]),
                       metallicFactor=met, roughnessFactor=rough, name=name, **kw)


cache = {}
for f in sorted(os.listdir(SRC)):
    if not f.endswith('.glb'):
        continue
    sc = trimesh.load(os.path.join(SRC, f), force='scene')
    stat = {}
    for node in sc.graph.nodes_geometry:
        _, gname = sc.graph[node]
        geo = sc.geometry[gname]

        oc = orig_color(geo)
        alpha = 1.0
        rho = lookup_density(node)
        is_acrylic = (rho is not None and ACRYLIC_RHO[0] <= rho <= ACRYLIC_RHO[1]
                      and re.search(ACRYLIC_NAME, node))
        # 亚克力优先级最高: 它必须透明, 不能被其它规则盖掉
        if is_acrylic:
            key = '亚克力护板 透明'
            (rgb, met, rough, alpha) = ACRYLIC_MAT
        # 饱和度次之: 镜头、灯条、指示灯保留其色相
        elif oc is not None and (oc.max() - oc.min()) > SAT_KEEP:
            key = f'保留原色{tuple(oc.astype(int))}'
            rgb, met, rough = tuple(oc / 255.0), 0.25, 0.45
        else:
            spec, note = override_of(node)
            if spec is not None:
                key = note
                rgb, met, rough, alpha = spec
            else:
                rho = lookup_density(node)
                key = material_of(rho) if rho else '默认'
                rgb, met, rough = PALETTE_SRGB[key]

        stat[key] = stat.get(key, 0) + 1
        if key not in cache:
            cache[key] = make(rgb, met, rough, key, alpha)
        geo.visual.material = cache[key]

    sc.export(os.path.join(DST, f))
    tag = '  '.join(f"{k}:{v}" for k, v in sorted(stat.items(), key=lambda x: -x[1])[:7])
    print(f"{f:20s} {tag}")
