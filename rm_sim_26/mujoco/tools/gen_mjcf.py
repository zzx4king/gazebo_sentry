#!/usr/bin/env python3
"""由 models/sentry/model.sdf 生成 MuJoCo MJCF.

惯量/位姿从 SDF 读取而非手抄, 这样 Gazebo 与 MuJoCo 两个模型不会各自漂移。
改了 SDF 后重跑本脚本即可。

用法: python3 tools/gen_mjcf.py
"""
import json
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SDF = os.path.normpath(os.path.join(HERE, '..', '..', 'models', 'sentry', 'model.sdf'))
ASSETS = os.path.normpath(os.path.join(HERE, '..', 'models', 'assets'))
OUT = os.path.normpath(os.path.join(HERE, '..', 'models', 'sentry.xml'))

# MJCF 材质: (rgba, specular, shininess, reflectance)
MATERIALS = {
    'fiberglass': ((0.021, 0.023, 0.025, 1), 0.15, 0.25, 0.02),
    'carbon':     ((0.013, 0.014, 0.016, 1), 0.30, 0.45, 0.05),
    'printed':    ((0.030, 0.031, 0.034, 1), 0.08, 0.12, 0.00),
    'aluminium':  ((0.340, 0.353, 0.372, 1), 0.85, 0.75, 0.35),
    'steel':      ((0.248, 0.258, 0.274, 1), 0.90, 0.85, 0.45),
    'brass':      ((0.272, 0.146, 0.052, 1), 0.85, 0.70, 0.30),
    'roller':     ((0.750, 0.750, 0.715, 1), 0.20, 0.25, 0.02),
    'camera':     ((0.340, 0.353, 0.372, 1), 0.75, 0.70, 0.30),
    'referee':    ((0.610, 0.625, 0.650, 1), 0.25, 0.30, 0.05),
    'lightbar':   ((0.700, 0.715, 0.750, 1), 0.30, 0.40, 0.10),
    'lidar':      ((0.448, 0.472, 0.514, 1), 0.60, 0.55, 0.20),
    'acrylic':    ((0.710, 0.780, 0.820, 0.22), 0.95, 0.90, 0.50),
    'pcb':        ((0.010, 0.062, 0.017, 1), 0.20, 0.35, 0.05),
    'fluor':      ((0.480, 0.760, 0.115, 1), 0.30, 0.45, 0.10),
    'accent':     ((0.180, 0.130, 0.010, 1), 0.55, 0.60, 0.15),
    'darkbox':    ((0.070, 0.074, 0.079, 1), 0.20, 0.30, 0.03),
    'misc':       ((0.024, 0.026, 0.029, 1), 0.15, 0.25, 0.02),
}


def parse_pose(el):
    if el is None or not (el.text or '').strip():
        return np.zeros(3), np.zeros(3)
    v = [float(x) for x in el.text.split()]
    return np.array(v[:3]), np.array(v[3:6] if len(v) >= 6 else [0, 0, 0])


def rpy_to_quat(r, p, y):
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    return np.array([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])


def fmt(a, n=6):
    return ' '.join(f'{float(x):.{n}g}' for x in np.atleast_1d(a))


def load_sdf():
    model = ET.parse(SDF).getroot().find('model')
    links, joints = {}, {}
    for l in model.findall('link'):
        pose_el = l.find('pose')
        t, r = parse_pose(pose_el)
        inert = l.find('inertial')
        m, com, I = 0.0, np.zeros(3), np.zeros(3)
        if inert is not None:
            if inert.find('mass') is not None:
                m = float(inert.find('mass').text)
            com, _ = parse_pose(inert.find('pose'))
            it = inert.find('inertia')
            if it is not None:
                I = np.array([float(it.find(k).text) for k in ('ixx', 'iyy', 'izz')])
        links[l.get('name')] = dict(
            t=t, r=r, rel=pose_el.get('relative_to') if pose_el is not None else None,
            m=m, com=com, I=I)
    for j in model.findall('joint'):
        pose_el = j.find('pose')
        t, r = parse_pose(pose_el)
        child = j.find('child').text
        ax = j.find('axis')
        xyz = np.array([float(x) for x in ax.find('xyz').text.split()]) if ax is not None else None
        lim = ax.find('limit') if ax is not None else None
        lo = float(lim.find('lower').text) if lim is not None and lim.find('lower') is not None else None
        hi = float(lim.find('upper').text) if lim is not None and lim.find('upper') is not None else None
        dmp = ax.find('dynamics/damping') if ax is not None else None
        joints[j.get('name')] = dict(
            parent=j.find('parent').text, child=child, t=t, r=r,
            rel=(pose_el.get('relative_to') if pose_el is not None else child),
            axis=xyz, type=j.get('type'), lower=lo, upper=hi,
            damping=float(dmp.text) if dmp is not None else 0.0)
    return links, joints


def resolve(links, joints, key, kind, cache):
    """返回实体坐标系在 base_link 系下的 (t, R) —— 关节角为 0 时"""
    ck = (kind, key)
    if ck in cache:
        return cache[ck]
    obj = links[key] if kind == 'link' else joints[key]
    rel = obj['rel']
    if rel is None:
        bt, bR = np.zeros(3), np.eye(3)
    elif rel in links:
        bt, bR = resolve(links, joints, rel, 'link', cache)
    elif rel in joints:
        bt, bR = resolve(links, joints, rel, 'joint', cache)
    else:
        bt, bR = np.zeros(3), np.eye(3)
    q = rpy_to_quat(*obj['r'])
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    out = (bt + bR @ obj['t'], bR @ R)
    cache[ck] = out
    return out


def quat_z_to(target):
    """返回把本体 +Z 转到 target 方向的四元数.

    枪口 site 的 +Z 就是发射方向, 必须与实测膛线轴严格对齐。早先用
    rpy(-90°,0,0) 得到的是纯 +Y, 而真实膛线是 (0.027, 0.997, 0.072) ——
    从送弹点到摩擦轮 45mm 上累积偏差 3.4mm, 超过夹持余量, 弹丸直接从
    轮子旁边溜过去, 加大压缩量也没用。
    """
    t = np.asarray(target, dtype=float)
    t = t / np.linalg.norm(t)
    z = np.array([0.0, 0.0, 1.0])
    d = float(z @ t)
    if d > 1 - 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if d < -1 + 1e-12:
        return np.array([0.0, 1.0, 0.0, 0.0])
    v = np.cross(z, t)
    w = 1.0 + d
    q = np.array([w, v[0], v[1], v[2]])
    return q / np.linalg.norm(q)


def mat_to_quat(R):
    tr = R.trace()
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                         (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                         (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    if i == 1:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                         0.25 * s, (R[1, 2] + R[2, 1]) / s])
    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                     (R[1, 2] + R[2, 1]) / s, 0.25 * s])


# ---- 视觉网格按 link 分组 (来自 glb_to_obj.py 的 manifest) ----
# omni_wheel 的网格四个轮子复用
LINK_MESH = {
    'base_link': 'base_link',
    'gimbal_link': 'gimbal_link',
    'small_yaw_link': 'small_yaw_link',
    'pitch_link': 'pitch_link',
}
# 网格自身坐标系是 SolidWorks 的 Y-up, MJCF 里绕 X 转 +90° 对齐 ROS Z-up
MESH_QUAT = rpy_to_quat(np.pi / 2, 0, 0)
# base_link 网格相对 link 原点还有一个平移 (与 SDF 的 visual pose 一致)
MESH_OFFSET = {'base_link': np.array([-0.0625, -0.0625, 0.0])}


def sub(parent, tag, **kw):
    el = ET.SubElement(parent, tag)
    for k, v in kw.items():
        el.set(k.replace('_', '-') if k.startswith('class_') else k, v)
    return el


def build():
    links, joints = load_sdf()
    cache = {}
    manifest = json.load(open(os.path.join(ASSETS, 'manifest.json'), encoding='utf-8'))
    # 弹仓腔壁 (tools/gen_mag_collision.py 生成)。缺文件不算错 —— 退回近似圆桶,
    # 只是容量会偏小, 会在末尾提示。
    magcol_path = os.path.join(ASSETS, 'magcol.json')
    magdata = (json.load(open(magcol_path, encoding='utf-8'))
               if os.path.exists(magcol_path) else {})
    magcol = magdata.get('meshes', [])
    mag_seeds = magdata.get('seeds', [])
    mag_capacity = int(magdata.get('capacity', MAG_CAPACITY))

    root = ET.Element('mujoco', model='sentry')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true',
                  meshdir='assets', texturedir='assets')

    # density/viscosity 开启流体力 —— 弹丸的阻力与马格努斯力靠它, 不手写弹道。
    # impratio=10 提高摩擦方向阻抗比(抗打滑), cone=elliptic 比默认金字塔锥准,
    # implicitfast 对 damping/armature 隐式积分, 比 RK4 稳且快。
    # 全向轮接触对这三个参数敏感, 取值参考 wuji-description 的标定模型。
    ET.SubElement(root, 'option', timestep='0.001', gravity='0 0 -9.81',
                  density='1.204', viscosity='1.8e-5',
                  integrator='implicitfast', solver='Newton',
                  iterations='50', tolerance='1e-10',
                  impratio='10', cone='elliptic', jacobian='sparse')
    ET.SubElement(root, 'size', memory='64M')

    vis = ET.SubElement(root, 'visual')
    ET.SubElement(vis, 'headlight', ambient='0.45 0.45 0.45',
                  diffuse='0.55 0.55 0.55', specular='0.2 0.2 0.2')
    ET.SubElement(vis, 'rgba', haze='0.87 0.90 0.94 1')
    # znear/zfar 之比决定深度缓冲精度。原来 0.01/60 是 6000:1, 精度严重不足,
    # 是阴影瑕疵与大面积错误着色的经典诱因(交互查看器上表现为地面整片发白,
    # 离屏渲染却复现不出来 —— 两条路径的深度处理不同)。收到 600:1。
    # shadowsize/offsamples 同步降一档, 减轻驱动压力, 视觉上几乎无差别。
    ET.SubElement(vis, 'quality', shadowsize='2048', offsamples='4')
    ET.SubElement(vis, 'map', znear='0.05', zfar='30')
    # 离屏帧缓冲默认 640x480, 超过就报错。放大以便截图/录像/相机传感器取图。
    ET.SubElement(vis, 'global', offwidth='1920', offheight='1080')

    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', type='skybox', builtin='gradient',
                  rgb1='0.62 0.68 0.76', rgb2='0.86 0.89 0.93', width='512', height='512')
    ET.SubElement(asset, 'texture', name='grid', type='2d', builtin='checker',
                  rgb1='0.30 0.32 0.35', rgb2='0.36 0.38 0.41',
                  width='512', height='512')
    ET.SubElement(asset, 'material', name='ground', texture='grid',
                  texrepeat='24 24', texuniform='true', reflectance='0')
    for name, (rgba, spec, shin, refl) in MATERIALS.items():
        ET.SubElement(asset, 'material', name=name, rgba=fmt(rgba),
                      specular=str(spec), shininess=str(shin), reflectance=str(refl))
    # 弹丸: 绿色荧光, 用 emission 让它自发光
    ET.SubElement(asset, 'material', name='tracer', rgba='0.35 1.0 0.15 1',
                  emission='0.85', specular='0.4', shininess='0.6')

    seen = set()
    for link, entries in manifest.items():
        for e in entries:
            mesh_name = e['file'][:-4]
            if mesh_name in seen:
                continue
            seen.add(mesh_name)
            ET.SubElement(asset, 'mesh', name=mesh_name, file=e['file'])
    for e in magcol:
        ET.SubElement(asset, 'mesh', name=e['file'][:-4], file=e['file'])

    # 嵌套 default: 辨识时改一个 class 即可扫过同类全部关节, 不必逐个 joint 改。
    # group 约定: 1=visual  2=collision  3=site
    default = ET.SubElement(root, 'default')
    sentry = ET.SubElement(default, 'default')
    sentry.set('class', 'sentry')

    d_vis = ET.SubElement(sentry, 'default')
    d_vis.set('class', 'visual')
    ET.SubElement(d_vis, 'geom', group='1', contype='0', conaffinity='0', density='0')

    # 碰撞体放 group 3: MuJoCo 查看器默认只显示 group 0/1/2, 放 2 的话这些粗包络
    # (底盘 r=0.30、云台 r=0.26 的圆柱)会半透明地罩在模型外面, 整车看着是橙色的。
    # 需要看碰撞体时在查看器的 Group enable 面板里勾上 3 即可。
    d_col = ET.SubElement(sentry, 'default')
    d_col.set('class', 'collision')
    ET.SubElement(d_col, 'geom', group='3', rgba='0.9 0.4 0.2 0.25',
                  condim='4', friction='1.0 0.02 0.001')

    # 云台三轴: armature 是折算转子惯量 N^2*J_rotor, 这里是待辨识量的初值。
    # 大yaw同步带 1:1 (CAD 实测), 故 N 只来自 DM4310 自身减速比。
    d_gim = ET.SubElement(sentry, 'default')
    d_gim.set('class', 'gimbal')
    ET.SubElement(d_gim, 'joint', armature=str(ARMATURE_GIMBAL),
                  damping='0.0', frictionloss='0.0')

    # 全向轮: 切向抓地、轴向近乎自由, 用各向异性摩擦近似辊子
    d_wh = ET.SubElement(sentry, 'default')
    d_wh.set('class', 'wheel')
    ET.SubElement(d_wh, 'joint', armature=str(ARMATURE_WHEEL),
                  damping='0.0', frictionloss='0.0')
    ET.SubElement(d_wh, 'geom', condim='4', friction='1.4 0.005 0.0002',
                  priority='1')

    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='12 12 0.05',
                  material='ground', condim='4', friction='1.0 0.02 0.001')
    ET.SubElement(world, 'light', pos='0 0 4.5', dir='0 0 -1',
                  directional='true', diffuse='0.5 0.5 0.5')
    ET.SubElement(world, 'light', pos='3 -3 3', dir='-0.6 0.6 -0.7',
                  directional='true', diffuse='0.35 0.35 0.35')

    def add_visual(body, link_key, scale_ok=True):
        for e in manifest.get(link_key, []):
            g = ET.SubElement(body, 'geom')
            g.set('class', 'visual')
            g.set('type', 'mesh')
            g.set('mesh', e['file'][:-4])
            g.set('material', e['material'])
            g.set('quat', fmt(MESH_QUAT))
            off = MESH_OFFSET.get(link_key)
            if off is not None:
                g.set('pos', fmt(off))

    def add_inertial(body, L):
        if L['m'] <= 0:
            return
        ET.SubElement(body, 'inertial', pos=fmt(L['com']), mass=f"{L['m']:.6g}",
                      diaginertia=fmt(L['I'], 6))

    # ---------- base ----------
    base = ET.SubElement(world, 'body', name='base_link', pos='0 0 0.30')
    base.set('childclass', 'sentry')
    ET.SubElement(base, 'freejoint', name='root')
    add_inertial(base, links['base_link'])
    add_visual(base, 'base_link')
    cg = ET.SubElement(base, 'geom', name='base_col', type='cylinder',
                       size='0.30 0.13', pos='0 0 0.10')
    cg.set('class', 'collision')

    # ---------- wheels ----------
    for jn in ('wheel_fl_joint', 'wheel_rl_joint', 'wheel_rr_joint', 'wheel_fr_joint'):
        J = joints[jn]
        L = links[J['child']]
        q = rpy_to_quat(*L['r'])
        wb = ET.SubElement(base, 'body', name=J['child'], pos=fmt(L['t']), quat=fmt(q))
        wb.set('childclass', 'wheel')
        add_inertial(wb, L)
        ET.SubElement(wb, 'joint', name=jn, type='hinge', axis=fmt(J['axis']),
                      damping=f"{J['damping']:.4g}")
        for e in manifest.get('omni_wheel', []):
            g = ET.SubElement(wb, 'geom')
            g.set('class', 'visual')
            g.set('type', 'mesh')
            g.set('mesh', e['file'][:-4])
            g.set('material', e['material'])
        # 全向轮接触: 用球近似, 各向异性摩擦模拟辊子(切向抓地/轴向自由)
        gc = ET.SubElement(wb, 'geom', name=f"{J['child']}_col", type='sphere',
                           size='0.076')
        gc.set('class', 'collision')

    # ---------- gimbal chain ----------
    def chain_body(parent_el, joint_name, mesh_key, extra=None):
        J = joints[joint_name]
        jt, jR = resolve(links, joints, joint_name, 'joint', cache)
        pt, pR = resolve(links, joints, J['parent'], 'link', cache)
        rel_t = pR.T @ (jt - pt)
        rel_R = pR.T @ jR
        b = ET.SubElement(parent_el, 'body', name=J['child'],
                          pos=fmt(rel_t), quat=fmt(mat_to_quat(rel_R)))
        add_inertial(b, links[J['child']])
        b.set('childclass', 'gimbal')
        kw = dict(name=joint_name, type='hinge', axis=fmt(J['axis']),
                  damping=f"{J['damping']:.4g}")
        if J['type'] == 'revolute' and J['lower'] is not None and abs(J['lower']) < 100:
            kw['range'] = f"{J['lower']:.4g} {J['upper']:.4g}"
        ET.SubElement(b, 'joint', **kw)
        if mesh_key:
            add_visual(b, mesh_key)
        return b

    gim = chain_body(base, 'big_yaw_joint', 'gimbal_link')
    gc2 = ET.SubElement(gim, 'geom', name='gimbal_col', type='cylinder',
                        size='0.26 0.17', pos='0 0 0.20')
    gc2.set('class', 'collision')

    # ---------- 送弹机构: 弹仓斗 + 拨弹盘 ----------
    # 位置取自 CAD: 拨弹盘中心 (-22.9, -27.7, 56.3) mm, 盘面法向竖直, 直径约 105mm,
    # 由下方 2006 电机驱动。弹仓在其上方, 弹丸靠自重落到盘上, 由拨齿带走。
    # 碰撞组用 4/4, 与弹丸(3/3)相交 -> 可碰; 与机体(1/1)不相交 -> 不干扰底盘接触。
    fx, fy, fz = FEEDER_POS
    # 弹仓斗单独建 body: 它若挂在 gimbal_link 上, 会被 proj<->gimbal_link 的
    # contact exclude 一并屏蔽, 弹丸直接穿过斗底掉到地上。
    hop = ET.SubElement(gim, 'body', name='hopper', pos='0 0 0')
    ET.SubElement(hop, 'inertial', pos=fmt(FEEDER_POS), mass='0.12',
                  diaginertia='4e-4 4e-4 6e-4')

    # 腔壁: 直接用 CAD 零件的凸包(tools/gen_mag_collision.py 生成)。
    # 曾经这里是手搓的 ⌀150x90mm 圆桶, 结果容量算成 84 发, 差了近一个数量级 ——
    # 真实腔体由 101 个零件围成, 形状根本不是圆桶。网格已在 gimbal_link 坐标系,
    # 所以 hopper body 的 pos 是 0。
    n_wall = 0
    for i, e in enumerate(magcol):
        g = ET.SubElement(hop, 'geom', name=f'magwall{i}', type='mesh',
                          mesh=e['file'][:-4], material='printed')
        g.set('contype', '4'); g.set('conaffinity', '4'); g.set('group', '3')
        n_wall += 1
    if n_wall == 0:                       # 没生成腔壁就退回圆桶, 至少别漏弹
        for i in range(16):
            a = 2 * np.pi * i / 16
            g = ET.SubElement(hop, 'geom', name=f'hopper_wall{i}', type='box',
                              size='0.003 0.016 ' + f'{HOPPER_H / 2:.4f}',
                              pos=fmt([fx + HOPPER_R * np.cos(a),
                                       fy + HOPPER_R * np.sin(a), fz + HOPPER_H / 2]),
                              quat=fmt(rpy_to_quat(0, 0, a)), material='printed')
            g.set('contype', '4'); g.set('conaffinity', '4'); g.set('group', '3')

    # 拨弹盘: 轮毂 + N 个径向拨齿, 绕竖直轴, 2006 电机驱动
    fb = ET.SubElement(gim, 'body', name='feeder', pos=fmt(FEEDER_POS))
    ET.SubElement(fb, 'inertial', pos='0 0 0', mass='0.045',
                  diaginertia='3.1e-5 3.1e-5 6.1e-5')
    ET.SubElement(fb, 'joint', name='feeder_joint', type='hinge', axis='0 0 1',
                  damping='1e-3', armature='2e-5')
    # 整盘而非小轮毂: 弹丸要坐在盘面上, 拨齿只负责带动。早先只做了 0.42R 的
    # 轮毂, 弹丸从轮毂与斗底环之间的空隙直接漏穿, 掉到地上。
    hub = ET.SubElement(fb, 'geom', name='feeder_hub', type='cylinder',
                        size=f'{FEEDER_R:.4f} 0.005', material='aluminium')
    hub.set('contype', '4'); hub.set('conaffinity', '4'); hub.set('group', '3')
    for i in range(FEEDER_PADDLES):
        a = 2 * np.pi * i / FEEDER_PADDLES
        pl = ET.SubElement(fb, 'geom', name=f'feeder_paddle{i}', type='box',
                           size=f'{FEEDER_R * 0.30:.4f} 0.0025 {FEEDER_PADDLE_H:.4f}',
                           pos=fmt([FEEDER_R * 0.68 * np.cos(a),
                                    FEEDER_R * 0.68 * np.sin(a), FEEDER_PADDLE_H]),
                           quat=fmt(rpy_to_quat(0, 0, a)), material='printed')
        pl.set('contype', '4'); pl.set('conaffinity', '4'); pl.set('group', '3')

    # 雷达: 必须是 body 才带质量 —— 用 site 会丢掉 0.265 kg, 直接影响大yaw惯量。
    # MuJoCo 里无 joint 的 body 即刚性固连, 等价于 SDF 的 fixed joint。
    L_lidar = links['livox_lidar']
    lt, lR = resolve(links, joints, 'livox_lidar', 'link', cache)
    gt, gR = resolve(links, joints, 'gimbal_link', 'link', cache)
    lidar_b = ET.SubElement(gim, 'body', name='livox_lidar',
                            pos=fmt(gR.T @ (lt - gt)),
                            quat=fmt(mat_to_quat(gR.T @ lR)))
    add_inertial(lidar_b, L_lidar)
    ET.SubElement(lidar_b, 'site', name='livox_lidar', size='0.012',
                  rgba='0.2 0.9 1.0 0.55')
    # 雷达视角相机: 沿 site 的 -Z 方向看出去
    ET.SubElement(lidar_b, 'camera', name='lidar_cam', pos='0 0 0',
                  quat=fmt(rpy_to_quat(0, 0, 0)), fovy='90')

    syaw = chain_body(gim, 'small_yaw_joint', 'small_yaw_link')
    pitch = chain_body(syaw, 'pitch_joint', 'pitch_link')

    # 枪口 site: 弹丸出膛点, site 的 +Z 即初速方向。
    # 位置取自 CAD —— RM18 裁判系统测速模块在 pitch_link 系的中心
    # (-23.0, 257.3, -54.3) mm。弹丸必须穿过该模块, 裁判系统靠它测初速;
    # 位置估错的话弹道看起来会从模块旁边飞出去。
    ET.SubElement(pitch, 'site', name='muzzle',
                  pos=fmt(MUZZLE_POS),
                  quat=fmt(quat_z_to(CHRONO_AXIS)),
                  size='0.006', rgba='1 0.35 0.1 0.9')
    # 测速模块中心, 便于可视化确认弹道确实穿过它
    ET.SubElement(pitch, 'site', name='chrono', pos=fmt(CHRONO_POS),
                  size='0.009', rgba='0.2 1 0.4 0.35')

    # ---------- 摩擦轮 ----------
    # 位置与朝向来自 CAD 实测, 不是照常见布局猜的:
    #   3508安装板(发射) 是一块水平薄板(5mm), 法向沿 pitch 系 Z, 上面有两个
    #   Ø26.4mm 电机孔, 左右对称跨在膛线两侧, 轴心间距 72.9mm。
    #   => 电机轴竖直, 两轮【左右并排】夹住弹丸, 不是上下叠放。
    # 孔心(pitch系 ROS): 左 (-38.2, 69.0, 26.1)  右 (+34.7, 69.0, 26.1) mm
    left_c, right_c = _fric_centers()
    for name, c in (('fric_left', left_c), ('fric_right', right_c)):
        wb = ET.SubElement(pitch, 'body', name=name, pos=fmt(c))
        ET.SubElement(wb, 'inertial', pos='0 0 0', mass=f'{FRIC_MASS}',
                      diaginertia=fmt([1.78e-5, 1.78e-5, 3.08e-5]))
        ET.SubElement(wb, 'joint', name=f'{name}_joint', type='hinge',
                      axis=fmt(FRIC_AXIS), damping='2e-4',
                      armature=str(FRIC_ARMATURE))
        # 圆柱默认轴向沿 Z, 与电机轴一致, 无需再转
        gv = ET.SubElement(wb, 'geom', name=f'{name}_geom', type='cylinder',
                           size=f'{FRIC_R} {FRIC_W / 2}',
                           material='roller', mass=f'{FRIC_MASS}',
                           condim='4', priority='2',
                           friction='2.2 0.01 0.0005',
                           solref='0.002 1', solimp='0.95 0.99 0.0005')
        gv.set('contype', '2')
        gv.set('conaffinity', '2')       # 只与弹丸(同组)碰, 不与机体碰
    # 自瞄相机 (第一人称视角)
    ET.SubElement(pitch, 'camera', name='gimbal_cam', pos='-0.0017 0.1395 0.0531',
                  quat=fmt(rpy_to_quat(np.pi / 2, 0, np.pi)), fovy='48')
    ET.SubElement(pitch, 'site', name='gimbal_imu', pos='-0.0017 0.0055 0.0306',
                  size='0.008', rgba='1 0.9 0.2 0.7')

    # ---------- 弹丸池 ----------
    pool = ET.SubElement(root, 'worldbody') if False else world
    for i in range(N_PROJECTILE):
        pb = ET.SubElement(pool, 'body', name=f'proj{i}', pos=f'0 0 {-5 - i * 0.1:.2f}')
        ET.SubElement(pb, 'freejoint', name=f'proj{i}_free')
        ET.SubElement(pb, 'geom', name=f'proj{i}_geom', type='sphere',
                      size=f'{PROJ_RADIUS}', mass=f'{PROJ_MASS}',
                      material='tracer', condim='4',
                      contype='7', conaffinity='7',
                      friction='2.2 0.01 0.0005', solref='0.002 1',
                      solimp='0.95 0.99 0.0005',
                      fluidshape='ellipsoid', fluidcoef='0.47 0.25 1.5 1.0 1.0')
        # 不给弹丸挂光源: 24 个 <light> 默认都投射阴影, 平时停在地下 z=-5,
        # 一发射就随弹丸进入场景, 阴影贴图被挤爆, 地面整片变白, 再发一发又恢复。
        # 荧光观感靠材质的 emission=0.85 就够, 不需要真光源。

    # ---------- 弹药账目 ----------
    # 写进 <custom> 而不是 Python 常量: 模型自包含, MJX 批量仿真和外部工具读同一份。
    # mag_capacity 是实测的自然堆积容量; mag_seed 是实体弹丸的初始堆放位置
    # (取自同一次实测的沉降结果, 离拨弹盘最近的那些点)。
    custom = ET.SubElement(root, 'custom')
    ET.SubElement(custom, 'numeric', name='mag_capacity', data=str(mag_capacity))
    if mag_seeds:
        flat = ' '.join(f'{v:.5f}' for p in mag_seeds[:N_PROJECTILE] for v in p)
        ET.SubElement(custom, 'numeric', name='mag_seed', data=flat)
    if magdata.get('cavity_lo'):
        # 腔体范围(gimbal_link 系), 用来判断仓内弹丸是否漏出去了
        ET.SubElement(custom, 'numeric', name='mag_cavity',
                      data=' '.join(f'{v:.5f}' for v in
                                    magdata['cavity_lo'] + magdata['cavity_hi']))

    # ---------- 相机 ----------
    ET.SubElement(world, 'camera', name='track_chase', mode='trackcom',
                  pos='-1.6 -1.6 1.15', xyaxes='0.707 -0.707 0 0.3 0.3 0.9')
    ET.SubElement(world, 'camera', name='track_side', mode='trackcom',
                  pos='0 -2.4 0.75', xyaxes='1 0 0 0 0.35 0.94')
    ET.SubElement(world, 'camera', name='track_top', mode='trackcom',
                  pos='0 -0.05 3.0', xyaxes='1 0 0 0 1 0')

    # ---------- 禁止机体自碰撞 ----------
    # 等价于 SDF 的 <self_collide>false</self_collide>。碰撞体是粗包络(底盘圆柱
    # r=0.30 把四个轮球都罩在里面, 又与云台圆柱重叠), 不屏蔽的话会凭空产生几十个
    # 接触约束跟关节较劲 —— 实测导致 big_yaw 辨识偏差 25%, 且难以察觉。
    # 弹丸不在此列, 仍可正常命中装甲。
    ROBOT_BODIES = ['base_link', 'gimbal_link', 'livox_lidar',
                    'small_yaw_link', 'pitch_link',
                    'wheel_fl_link', 'wheel_rl_link', 'wheel_rr_link', 'wheel_fr_link']
    contact = ET.SubElement(root, 'contact')
    for i, b1 in enumerate(ROBOT_BODIES):
        for b2 in ROBOT_BODIES[i + 1:]:
            ET.SubElement(contact, 'exclude', body1=b1, body2=b2)
    # 弹丸也要排除机体的粗碰撞包络: 枪口位于云台包络圆柱(r=0.26)内部, 不排除的话
    # 弹丸一生成就被这个非实体的包络弹飞, 根本到不了摩擦轮(实测 67 次接触,
    # 弹丸被推到离膛线轴 106mm 处)。摩擦轮不在此列, 仍正常夹持加速。
    for i in range(N_PROJECTILE):
        for b in ROBOT_BODIES:
            ET.SubElement(contact, 'exclude', body1=f'proj{i}', body2=b)
    # 弹仓腔壁与拨弹盘互不碰撞: 出弹导轨、2006 安装座就贴在盘边上(轴心 7mm 处),
    # 真车靠轴承和间隙转得动, 建模成接触只会把拨弹盘卡死。两者仍各自与弹丸碰撞。
    ET.SubElement(contact, 'exclude', body1='hopper', body2='feeder')

    # ---------- 执行器 ----------
    act = ET.SubElement(root, 'actuator')
    for name, gear, ctrl in (('big_yaw_joint', 1, 12.0),
                             ('small_yaw_joint', 1, 6.0),
                             ('pitch_joint', 1, 6.0)):
        ET.SubElement(act, 'motor', name=f'{name}_trq', joint=name,
                      gear=str(gear), ctrlrange=f'-{ctrl} {ctrl}')
    for w in ('wheel_fl_joint', 'wheel_rl_joint', 'wheel_rr_joint', 'wheel_fr_joint'):
        ET.SubElement(act, 'motor', name=f'{w}_trq', joint=w,
                      gear='1', ctrlrange='-6 6')
    # 摩擦轮用速度伺服: 真车是电调闭环控转速, 这里同构
    for w in ('fric_left_joint', 'fric_right_joint'):
        ET.SubElement(act, 'velocity', name=f'{w}_vel', joint=w,
                      kv='0.05', ctrlrange='-1400 1400')
    # 拨弹盘: 2006 电机, 速度伺服。转速决定供弹率, 是可辨识量。
    # kv 要够大才能带动压在盘上的整仓弹丸: kv=0.02 时 90 发的重量把转速压到
    # 0.4 rad/s(目标 12)。2006 减速后堵转扭矩约 1 N·m, 这里取 0.5 与之相称。
    ET.SubElement(act, 'velocity', name='feeder_joint_vel', joint='feeder_joint',
                  kv='0.5', ctrlrange='-60 60')

    # ---------- 传感器 (辨识用) ----------
    sen = ET.SubElement(root, 'sensor')
    for j in ('big_yaw_joint', 'small_yaw_joint', 'pitch_joint',
              'wheel_fl_joint', 'wheel_rl_joint', 'wheel_rr_joint', 'wheel_fr_joint'):
        ET.SubElement(sen, 'jointpos', name=f'{j}_pos', joint=j)
        ET.SubElement(sen, 'jointvel', name=f'{j}_vel', joint=j)
    for j in ('big_yaw_joint', 'small_yaw_joint', 'pitch_joint'):
        ET.SubElement(sen, 'jointactuatorfrc', name=f'{j}_frc', joint=j)
    ET.SubElement(sen, 'framequat', name='imu_quat', objtype='site', objname='gimbal_imu')
    ET.SubElement(sen, 'gyro', name='imu_gyro', site='gimbal_imu')
    ET.SubElement(sen, 'accelerometer', name='imu_acc', site='gimbal_imu')
    ET.SubElement(sen, 'framepos', name='muzzle_pos', objtype='site', objname='muzzle')
    for w in ('fric_left_joint', 'fric_right_joint', 'feeder_joint'):
        ET.SubElement(sen, 'jointvel', name=f'{w}_vel', joint=w)

    xml = minidom.parseString(ET.tostring(root, 'utf-8')).toprettyxml(indent='  ')
    xml = '\n'.join(line for line in xml.split('\n') if line.strip())
    with open(OUT, 'w', encoding='utf-8') as fh:
        fh.write(xml)
    print(f'写入 {OUT}')
    print(f'  link: {len(links)}  joint: {len(joints)}  弹丸池: {N_PROJECTILE}')


# armature = 折算转子惯量 N^2*J_rotor, 待辨识量的初值; 0 则退化为纯 CAD 刚体
ARMATURE_GIMBAL = 0.004
ARMATURE_WHEEL = 0.002

# RM18 裁判系统测速模块在 pitch_link 系的中心 (CAD 实测, ROS 米制)。
# 弹丸必须沿该模块的中心线穿过 —— 真车靠它测初速, 位置或方向偏了都不对。
# 模块本体 Z 轴在 pitch 系映射为 (0,-1,0), 故中心线沿 ±Y, 发射方向为 +Y。
# 由 pitch_link 网格实测: 取测速模块全部顶点的质心与主轴。
# 早先用单个子零件的装配变换算, 结果偏了 117 mm, 标记点直接落在模型外。
CHRONO_POS = np.array([-0.0036, 0.1595, 0.0076])
CHRONO_AXIS = np.array([0.0271, 0.9971, 0.0716])
# 枪口置于膛线上、摩擦轮之后。摩擦轮沿膛线在 -89mm 处, 枪口取 -70mm(轮出口侧)
MUZZLE_POS = CHRONO_POS - CHRONO_AXIS * 0.070

# 摩擦轮 (CAD: 57 g, ρ=1602 聚氨酯)
# 安装位置由 3508安装板(发射) 上的两个电机孔实测得到, 见下方注释。
FRIC_AXIS = np.array([0.0, 0.0, 1.0])              # 轴竖直 (板面法向)
FRIC_Y = 0.0690           # 轮心沿膛线的位置 (孔心实测)
FRIC_SPAN = 0.0729        # 两轮轴心间距 (孔心实测)
FRIC_GAP = 0.0158         # 夹持间隙, 弹丸压缩约 1.2 mm
FRIC_R = (FRIC_SPAN - FRIC_GAP) / 2               # = 28.6 mm


def _fric_centers():
    """两轮心: 沿膛线取孔心的轴向位置, 横向按实测间距对称跨在膛线两侧.

    孔心只定了电机轴在水平面内的位置 —— 轴是竖直的, 轮子在轴上的高度自由。
    直接照搬板面高度(26.1mm)会让膛线从两轮下方 25mm 处穿过, 弹丸只蹭到一侧,
    实测弹速掉到 6.6 m/s(效率 27%)。所以轮心必须落在膛线上。
    """
    t = (FRIC_Y - CHRONO_POS[1]) / CHRONO_AXIS[1]
    on_bore = CHRONO_POS + CHRONO_AXIS * t
    lateral = np.cross(CHRONO_AXIS, FRIC_AXIS)
    lateral = lateral / np.linalg.norm(lateral)
    return on_bore - lateral * FRIC_SPAN / 2, on_bore + lateral * FRIC_SPAN / 2
FRIC_W = 0.016            # 轮宽 m
FRIC_MASS = 0.057
FRIC_ARMATURE = 4.0e-5    # 3508 转子折算惯量, 待辨识

# ---- 送弹机构 (CAD 实测, gimbal_link 系 ROS 米制) ----
# 拨弹盘: 水平圆盘绕竖直轴转, 由下方 2006 电机驱动; 弹仓在其上方
FEEDER_POS = np.array([-0.0229, -0.0277, 0.0563])
FEEDER_R = 0.052          # 盘半径 (包围盒 103~110mm)
FEEDER_PADDLES = 8        # 拨齿数
FEEDER_PADDLE_H = 0.011   # 拨齿高度, 略低于弹丸直径以便单发拨出
HOPPER_R = 0.075          # 仅在缺 magcol.json 时兜底用的近似圆桶内半径
HOPPER_H = 0.090          # 同上
N_MAGAZINE = 20           # 预装弹丸数

# 弹仓自然堆积容量, 实测值。tools/measure_mag_capacity.py 复现:
#   云台层 155 个零件全部做碰撞体 -> 18mm 格点找出 2787 个不与结构干涉的位置
#   -> 全部生成弹丸只加重力沉降 -> 仓内 711 发, 6 秒后不再变化
#   -> 从仓顶再补灌 784 发 -> 714 发, 即已饱和 (三次重复 711/712/714)
# 别拿包围盒体积估: 弹仓 AABB 有 27.2 L, 按堆积率能算出五千发, 但那个盒子里
# 塞满了 NUC、C 板、云台支撑, 真实空腔只是其中一小块。
MAG_CAPACITY = 712

# 物理弹丸池大小。实测步进代价(见上述工具的性能测量):
#   90 发在仓内 -> 1.16x 实时;  300 发 -> 0.09x;  712 发 -> 0.11x
# 所以整仓 712 发不可能全用刚体。池子只覆盖拨弹盘附近的实体弹丸, 其余走
# ProjectilePool 的储备计数 —— 弹药账目仍是 712 发。
N_PROJECTILE = 90
PROJ_RADIUS = 0.0085      # 17mm 弹丸
PROJ_MASS = 0.0032        # 官方 17mm 弹丸 3.2 g

if __name__ == '__main__':
    build()
