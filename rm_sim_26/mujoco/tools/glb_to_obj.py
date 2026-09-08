#!/usr/bin/env python3
"""把 models/sentry/meshes/*.glb 按材料分组导出为 OBJ, 供 MuJoCo 使用.

MuJoCo 只读 OBJ/STL, 不认 GLB, 且一个 mesh 只能有一种材质。所以按材料把零件
分组合并: 每个 (link, 材料) 出一个 OBJ, MJCF 里再给每组挂对应的 material。
分组同时大幅降低 geom 数量 —— 逐零件建 geom 会有近千个, 拖垮渲染。

用法: python3 tools/glb_to_obj.py [--decimate 0.5]
"""
import argparse
import json
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, '..', '..', 'models', 'sentry', 'meshes'))
DST = os.path.normpath(os.path.join(HERE, '..', 'models', 'assets'))

# GLB 里的材质名 -> MJCF 材质名 (稳定的短标识, 中文不适合做文件名)
MAT_SLUG = {
    '玻纤': 'fiberglass',
    '碳纤维': 'carbon',
    '打印': 'printed',
    '铝': 'aluminium',
    '钢': 'steel',
    '铜': 'brass',
    '默认': 'misc',
    '亚克力护板 透明': 'acrylic',
    '全向轮辊子 白色聚氨酯': 'roller',
    '海康工业相机 银灰铝': 'camera',
    '相机': 'camera',
    '裁判系统测速模块 白塑料': 'referee',
    '裁判系统装甲模块': 'referee',
    '裁判系统灯条': 'lightbar',
    '裁判系统场地交互': 'referee',
    '裁判系统主控': 'referee',
    '裁判系统定位模块': 'referee',
    '电源管理模块': 'darkbox',
    'Livox Mid360 机身': 'lidar',
    '荧光充能': 'fluor',
    'C板 PCB 墨绿': 'pcb',
}


def slug_of(mat_name):
    if not mat_name:
        return 'misc'
    if mat_name in MAT_SLUG:
        return MAT_SLUG[mat_name]
    if mat_name.startswith('保留原色'):
        return 'accent'          # 镜头/灯条等保留原色的件, 统一走一个强调色
    return 'misc'


def base_color(mat):
    c = getattr(mat, 'baseColorFactor', None)
    if c is None:
        return (0.2, 0.2, 0.2, 1.0)
    a = np.array(c, dtype=float)
    if a.max() > 1.5:
        a = a / 255.0
    return tuple(float(x) for x in a[:4])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--decimate', type=float, default=0.0,
                    help='目标面数比例 (0=不简化). 需要 open3d/fast-simplification')
    args = ap.parse_args()

    os.makedirs(DST, exist_ok=True)
    manifest = {}

    for fname in sorted(os.listdir(SRC)):
        if not fname.endswith('.glb'):
            continue
        link = fname[:-4]
        scene = trimesh.load(os.path.join(SRC, fname), force='scene')

        groups = {}
        colors = {}
        for node in scene.graph.nodes_geometry:
            T, gname = scene.graph[node]
            geo = scene.geometry[gname]
            mat = getattr(geo.visual, 'material', None)
            slug = slug_of(getattr(mat, 'name', None))
            m = geo.copy()
            m.apply_transform(T)
            groups.setdefault(slug, []).append(m)
            if mat is not None:
                colors.setdefault(slug, base_color(mat))

        entries = []
        for slug, meshes in sorted(groups.items()):
            merged = trimesh.util.concatenate(meshes)
            if args.decimate and 0 < args.decimate < 1:
                target = int(len(merged.faces) * args.decimate)
                try:
                    merged = merged.simplify_quadric_decimation(target)
                except Exception as exc:              # noqa: BLE001
                    print(f'  简化失败({slug}): {exc}', file=sys.stderr)
            out = f'{link}__{slug}.obj'
            merged.export(os.path.join(DST, out), include_texture=False)
            entries.append({'file': out, 'material': slug,
                            'rgba': colors.get(slug, (0.2, 0.2, 0.2, 1.0)),
                            'faces': int(len(merged.faces))})
            print(f'{link:18s} {slug:12s} {len(merged.faces):>7d} 面  -> {out}')
        manifest[link] = entries

    with open(os.path.join(DST, 'manifest.json'), 'w', encoding='utf-8') as fh:
        json.dump(manifest, fh, indent=1, ensure_ascii=False)
    total = sum(e['faces'] for v in manifest.values() for e in v)
    print(f'\n共 {sum(len(v) for v in manifest.values())} 个 OBJ, {total} 面')


if __name__ == '__main__':
    main()
