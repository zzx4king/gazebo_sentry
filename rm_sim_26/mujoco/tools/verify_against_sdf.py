#!/usr/bin/env python3
"""交叉校验 MJCF 与源 SDF 是否一致.

对比两项:
  1) 总质量
  2) 各关节的等效惯量 —— MuJoCo 取质量矩阵对角元 M[i,i], SDF 侧由
     scripts/check_inertia.py 用平行轴定理算。两条完全独立的路径, 数值
     对上才说明转换没丢东西。

用法: python3 tools/verify_against_sdf.py
"""
import os
import subprocess
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MJCF = os.path.normpath(os.path.join(HERE, '..', 'models', 'sentry.xml'))
SDF = os.path.normpath(os.path.join(HERE, '..', '..', 'models', 'sentry', 'model.sdf'))
CHECKER = os.path.normpath(os.path.join(HERE, '..', '..', 'scripts', 'check_inertia.py'))

TOL_REL = 0.01      # 1% 以内视为一致


def sdf_values():
    out = subprocess.run([sys.executable, CHECKER, SDF],
                         capture_output=True, text=True, check=True).stdout
    vals, total = {}, None
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 5 and parts[0].endswith('_joint'):
            vals[parts[0]] = float(parts[-2])
        if line.startswith('整车质量'):
            total = float(line.split(':')[1].split()[0])
    return vals, total


# MJCF 独有、源 SDF 中不存在的 body。摩擦轮在 CAD 总装里没被插入(零件文件有,
# 装配体没引用), 但真车确实装了, 所以 MJCF 里建了 —— 比对时临时移除, 否则
# pitch 惯量会因这 114 g 差出 30%, 掩盖真正的转换错误。
MJCF_ONLY_BODIES = ('fric_left', 'fric_right', 'feeder', 'hopper')


def load_comparable(path: str) -> mujoco.MjModel:
    """移除 MJCF 独有的 body, 得到与 SDF 可比的模型"""
    spec = mujoco.MjSpec.from_file(path)
    for name in MJCF_ONLY_BODIES:
        try:
            spec.delete(spec.body(name))
        except (KeyError, ValueError):
            pass
    return spec.compile()


def main():
    m = load_comparable(MJCF)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    # 弹丸不属于机器人, 单独扣除
    proj_mass = sum(m.body_mass[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'proj{i}')]
                    for i in range(sum(1 for i in range(m.nbody)
                                       if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
                                       and mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i).startswith('proj'))))
    robot_mass = float(sum(m.body_mass)) - proj_mass

    # mujoco >=3.11 是 mj_fullM(m, d, dst); 旧版是 mj_fullM(m, dst, d.qM)
    M = np.zeros((m.nv, m.nv))
    try:
        mujoco.mj_fullM(m, d, M)
    except TypeError:
        mujoco.mj_fullM(m, M, d.qM)

    sdf_j, sdf_total = sdf_values()

    print(f"{'项':<20} {'MJCF刚体':>12} {'SDF':>12} {'相对差':>9} {'armature':>10}")
    print('(MJCF 列已扣除 armature —— 那是折算转子惯量, SDF 侧不含该项)\n')
    ok = True

    rel = abs(robot_mass - sdf_total) / sdf_total
    ok &= rel < TOL_REL
    print(f"{'总质量 kg':<20} {robot_mass:>12.4f} {sdf_total:>12.4f} {rel * 100:>8.2f}%")

    for jn, sdf_v in sdf_j.items():
        ji = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if ji < 0:
            print(f"{jn:<20} {'缺失':>12}")
            ok = False
            continue
        dof = m.jnt_dofadr[ji]
        armature = float(m.dof_armature[dof])
        mj_v = float(M[dof, dof]) - armature      # 扣掉转子折算项, 只比刚体部分
        rel = abs(mj_v - sdf_v) / max(sdf_v, 1e-9)
        ok &= rel < TOL_REL
        print(f"{jn:<20} {mj_v:>12.5f} {sdf_v:>12.5f} {rel * 100:>8.2f}% {armature:>10.5f}")

    print()
    if ok:
        print('一致 (阈值 1%)')
        return 0
    print('不一致 —— MJCF 与 SDF 存在差异, 检查上表')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
