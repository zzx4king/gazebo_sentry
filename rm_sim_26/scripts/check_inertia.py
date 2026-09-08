#!/usr/bin/env python3
"""从 model.sdf 解析各 link, 计算各转轴的等效惯量 J (仿真里实际生效的值).

对每个关节, J = Σ(该关节下游所有 link) 关于转轴的惯量。
用于核对 SDF 是否真的表达了预期的动力学参数。
"""
import sys
import xml.etree.ElementTree as ET

import numpy as np

SDF = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/neomelt/sentry_nav_26/src/rm_sim_26/models/sentry/model.sdf'

root = ET.parse(SDF).getroot()
model = root.find('model')


def parse_pose(el):
    if el is None or el.text is None:
        return np.zeros(3), np.zeros(3)
    v = [float(x) for x in el.text.split()]
    return np.array(v[:3]), np.array(v[3:6] if len(v) >= 6 else [0, 0, 0])


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


links, joints = {}, {}
for l in model.findall('link'):
    name = l.get('name')
    t, r = parse_pose(l.find('pose'))
    rel = l.find('pose').get('relative_to') if l.find('pose') is not None else None
    inert = l.find('inertial')
    m = 0.0
    com = np.zeros(3)
    I = np.zeros(3)
    if inert is not None:
        m = float(inert.find('mass').text) if inert.find('mass') is not None else 0.0
        com, _ = parse_pose(inert.find('pose'))
        it = inert.find('inertia')
        if it is not None:
            I = np.array([float(it.find(k).text) for k in ('ixx', 'iyy', 'izz')])
    links[name] = dict(t=t, r=r, rel=rel, m=m, com=com, I=I)

for j in model.findall('joint'):
    name = j.get('name')
    child = j.find('child').text
    pose_el = j.find('pose')
    t, r = parse_pose(pose_el)
    # SDF 规范: joint 无 <pose> 时, 其坐标系默认为【子 link 坐标系】
    rel = pose_el.get('relative_to') if pose_el is not None else child
    if pose_el is not None and rel is None:
        rel = child
    ax = j.find('axis')
    xyz = np.array([float(x) for x in ax.find('xyz').text.split()]) if ax is not None else None
    joints[name] = dict(parent=j.find('parent').text, child=child,
                        t=t, r=r, rel=rel, axis=xyz, type=j.get('type'))

# 解析每个 link 在 base_link 系下的位姿 (关节角=0)
frames = {'base_link': (np.zeros(3), np.eye(3))}


def frame_of(key, kind):
    """kind: 'link' or 'joint'; 返回该实体坐标系在 base 系的 (t, R)"""
    cache_key = (kind, key)
    if cache_key in frames:
        return frames[cache_key]
    obj = links[key] if kind == 'link' else joints[key]
    rel = obj['rel']
    if rel is None:
        bt, bR = np.zeros(3), np.eye(3)
    elif rel in links:
        bt, bR = frame_of(rel, 'link')
    elif rel in joints:
        bt, bR = frame_of(rel, 'joint')
    else:
        bt, bR = np.zeros(3), np.eye(3)
    R = bR @ rpy_mat(*obj['r'])
    t = bt + bR @ obj['t']
    frames[cache_key] = (t, R)
    return t, R


frames[('link', 'base_link')] = (np.zeros(3), np.eye(3))

# 关节树: 找每个关节的下游 link 集合
children = {}
for jn, j in joints.items():
    children.setdefault(j['parent'], []).append((jn, j['child']))


def downstream(link_name):
    out = [link_name]
    for jn, ch in children.get(link_name, []):
        out += downstream(ch)
    return out


print(f"{'关节':<18} {'类型':<11} {'下游link':<42} {'J (kg·m²)':>10}  质量kg")
for jn, j in joints.items():
    if j['type'] not in ('revolute', 'continuous'):
        continue
    jt, jR = frame_of(jn, 'joint')
    axis_w = jR @ j['axis']
    axis_w = axis_w / np.linalg.norm(axis_w)
    tot_J, tot_m = 0.0, 0.0
    names = downstream(j['child'])
    for ln in names:
        L = links[ln]
        lt, lR = frame_of(ln, 'link')
        com_w = lt + lR @ L['com']
        # 惯量张量(对角, link系) 转到世界
        Ib = np.diag(L['I'])
        Iw = lR @ Ib @ lR.T
        # 关于转轴的惯量: axis^T I axis + m * d_perp^2
        d = com_w - jt
        d_perp2 = np.dot(d, d) - np.dot(d, axis_w) ** 2
        tot_J += axis_w @ Iw @ axis_w + L['m'] * d_perp2
        tot_m += L['m']
    disp = ','.join(n.replace('_link', '') for n in names)
    print(f"{jn:<18} {j['type']:<11} {disp[:40]:<42} {tot_J:>10.5f}  {tot_m:.3f}")

print(f"\n整车质量(所有link): {sum(l['m'] for l in links.values()):.3f} kg")
