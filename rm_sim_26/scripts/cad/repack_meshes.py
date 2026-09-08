#!/usr/bin/env python3
"""重打包 GLB: 只保留被节点实例引用的网格, 剥掉未引用的数据."""
import os
import trimesh

SRC = 'meshes'
DST = 'meshes_slim'
os.makedirs(DST, exist_ok=True)

for f in sorted(os.listdir(SRC)):
    if not f.endswith('.glb'):
        continue
    sc = trimesh.load(os.path.join(SRC, f), force='scene')
    slim = trimesh.Scene()
    # 保留原节点名 —— 后续按零件名查 CAD 材料密度要用
    for i, node in enumerate(sc.graph.nodes_geometry):
        T, gname = sc.graph[node]
        slim.add_geometry(sc.geometry[gname], transform=T,
                          node_name=f'{node}#{i}', geom_name=gname)
    out = os.path.join(DST, f)
    slim.export(out)
    a = os.path.getsize(os.path.join(SRC, f)) / 1048576
    b = os.path.getsize(out) / 1048576
    print(f"{f:22s} {a:6.1f} -> {b:6.1f} MB  ({100*(1-b/a):4.1f}% 减)  "
          f"实例={len(sc.graph.nodes_geometry)}")
