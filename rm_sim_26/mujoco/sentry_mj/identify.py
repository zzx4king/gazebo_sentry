"""云台系统辨识.

辨识模型(单轴, 刚体):

    tau = J*ddq + b*dq + fc*sign(dq) + tau_g(q)

  J   等效惯量, 含折算转子惯量 N^2*J_rotor
  b   粘滞摩擦
  fc  库仑摩擦
  tau_g 重力力矩; yaw 轴竖直, 该项为 0, pitch 轴为 m*g*l*cos(q)

在仿真里做这件事的意义不在于"得到 J" —— 模型里的 J 本来就是已知的。意义在于
**先验证辨识流程本身**: 用已知真值的系统跑一遍, 若估计值回不到真值, 说明激励设计、
数值微分或最小二乘有问题, 那么拿到真车上得到的数同样不可信。

真值取自 mjModel: M[i,i] + armature。这是独立于辨识流程的参照。

用法
  python -m sentry_mj.identify --joint pitch_joint --mode chirp
  python -m sentry_mj.identify --all
"""
from __future__ import annotations

import argparse
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.normpath(os.path.join(HERE, '..', 'models', 'sentry.xml'))

JOINTS = ('big_yaw_joint', 'small_yaw_joint', 'pitch_joint')


# ---------------------------------------------------------------- 真值
def load_bench_model(path: str = MODEL) -> mujoco.MjModel:
    """加载"台架版"模型: 删掉底盘自由关节, 把车焊死在世界系。

    为什么必须在编译期删而不是运行时清零底盘速度: 自由底盘会被云台的反作用
    推着反向转, 云台感受到的等效惯量因此小于锁定值。运行时强行清零 qvel 治标
    不治本 —— qacc 仍是按自由底盘算出来的, 测量与真值对不上(实测 big_yaw
    辨识值 0.179 vs 锁定真值 0.241, 差 25%)。

    这对应真车做辨识时把车固定在台架上, 是同一件事。
    """
    spec = mujoco.MjSpec.from_file(path)
    for j in list(spec.body('base_link').joints):
        if j.type == mujoco.mjtJoint.mjJNT_FREE:
            spec.delete(j)
    return spec.compile()


def ground_truth(m: mujoco.MjModel, d: mujoco.MjData, joint: str) -> dict:
    """从 mjModel 读出该轴的真实参数, 作为辨识结果的参照"""
    mujoco.mj_forward(m, d)
    ji = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)
    dof = m.jnt_dofadr[ji]
    M = np.zeros((m.nv, m.nv))
    try:
        mujoco.mj_fullM(m, d, M)
    except TypeError:
        mujoco.mj_fullM(m, M, d.qM)
    return {
        'J': float(M[dof, dof]),          # 已含 armature
        'b': float(m.dof_damping[dof]),
        'fc': float(m.dof_frictionloss[dof]),
        'armature': float(m.dof_armature[dof]),
    }


# ---------------------------------------------------------------- 激励
def chirp(t: np.ndarray, f0: float, f1: float, amp: float) -> np.ndarray:
    """恒幅值线性调频. 仅用于无限位的轴(大yaw)。"""
    T = t[-1]
    k = (f1 - f0) / T
    return amp * np.sin(2 * np.pi * (f0 * t + 0.5 * k * t * t))


def chirp_shaped(t: np.ndarray, f0: float, f1: float,
                 j_nominal: float, excursion: float) -> np.ndarray:
    """恒位移扫频: 力矩幅值 ∝ f^2, 使位移幅值在全频段保持 excursion 不变。

    为什么必须这样: 二阶系统在力矩幅值恒定时, 位移幅值 ∝ 1/f^2。pitch 轴
    J≈0.019、限位跨度仅 0.9 rad, 用 1 N·m 恒幅值扫到 0.2 Hz 需要 33 rad 位移,
    结果 90% 的样本被顶在限位上, 约束力主导, 拟合必然发散(实测 R2=0.15、
    惯量为负)。

    j_nominal 只用来整形激励, 不参与拟合 —— 拟合仍是独立的最小二乘。
    """
    T = t[-1]
    k = (f1 - f0) / T
    f_inst = f0 + k * t
    omega = 2 * np.pi * f_inst
    return j_nominal * excursion * omega ** 2 * np.sin(
        2 * np.pi * (f0 * t + 0.5 * k * t * t))


def prbs(n: int, amp: float, hold: int, rng) -> np.ndarray:
    """伪随机二进制序列, 频谱平坦, 对非线性摩擦更鲁棒"""
    steps = int(np.ceil(n / hold))
    seq = rng.choice((-amp, amp), size=steps)
    return np.repeat(seq, hold)[:n]


# ---------------------------------------------------------------- 实验
def run_experiment(m, d, joint: str, tau: np.ndarray, lock_others: bool = True,
                   pin_base: bool = True):
    """给定力矩序列, 记录 q/dq/ddq/约束力.

    pin_base: 冻结底盘自由关节, 等价于真车做台架辨识时把车固定住。不冻结的话
    云台反作用会让整车晃动, 反力矩混进测量, 单轴模型不成立。
    其余云台轴用强 PD 钉住, 同理是为了消除耦合。
    """
    ji = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)
    dof = m.jnt_dofadr[ji]
    aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{joint}_trq')
    if aid < 0:
        raise RuntimeError(f'{joint} 没有对应的 actuator')

    other_aids = []
    if lock_others:
        for j in JOINTS:
            if j == joint:
                continue
            a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{j}_trq')
            if a >= 0:
                other_aids.append((a, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)))

    base_free = m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    base_qpos0 = d.qpos[:7].copy() if base_free else None

    n = len(tau)
    q = np.zeros(n)
    dq = np.zeros(n)
    ddq = np.zeros(n)
    tau_applied = np.zeros(n)
    fc_constraint = np.zeros(n)

    for i in range(n):
        d.ctrl[aid] = tau[i]
        # 其余云台轴用强 PD 钉住, 避免耦合污染单轴辨识
        for a, oj in other_aids:
            odof = m.jnt_dofadr[oj]
            d.ctrl[a] = -120.0 * d.qpos[m.jnt_qposadr[oj]] - 12.0 * d.qvel[odof]
        mujoco.mj_step(m, d)
        if pin_base and base_free:
            d.qpos[:7] = base_qpos0
            d.qvel[:6] = 0.0
        q[i] = d.qpos[m.jnt_qposadr[ji]]
        dq[i] = d.qvel[dof]
        ddq[i] = d.qacc[dof]
        tau_applied[i] = d.actuator_force[aid]
        fc_constraint[i] = d.qfrc_constraint[dof]
    return q, dq, ddq, tau_applied, fc_constraint


# ---------------------------------------------------------------- 拟合
def fit(q, dq, ddq, tau, with_gravity: bool = False, constraint=None):
    """最小二乘: tau = J*ddq + b*dq + fc*sign(dq) [+ mgl*cos(q)]

    两处样本剔除, 缺一不可:
      1) 低速样本 —— sign(dq) 在过零点病态, 会把 fc 带偏
      2) 触限样本 —— 关节顶在限位时约束力主导, 上式根本不成立。不剔除的话
         pitch 轴会拟合出负惯量。
    """
    mask = np.abs(dq) > 0.02
    if constraint is not None:
        at_limit = np.abs(constraint) > 1e-4
        mask &= ~at_limit
        if at_limit.mean() > 0.5:
            print(f'    [警告] {at_limit.mean() * 100:.0f}% 的样本触限, '
                  f'应减小激励幅值或改用恒位移扫频')
    if mask.sum() < 100:
        raise RuntimeError('有效样本太少, 加大激励幅值或延长时长')
    cols = [ddq[mask], dq[mask], np.sign(dq[mask])]
    names = ['J', 'b', 'fc']
    if with_gravity:
        cols.append(np.cos(q[mask]))
        names.append('mgl')
    A = np.column_stack(cols)
    y = tau[mask]
    theta, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - A @ theta
    r2 = 1.0 - resid.var() / y.var() if y.var() > 0 else float('nan')
    # 条件数: 反映各参数是否可分离, 过大说明激励不足
    cond = float(np.linalg.cond(A))
    return dict(zip(names, theta)), r2, cond, int(mask.sum())


# ---------------------------------------------------------------- 摩擦分离
def friction_sweep(joint: str, speeds=(0.15, 0.3, 0.6, 1.0, 1.6, 2.4),
                   settle: float = 1.2, hold: float = 0.8,
                   model_path: str = MODEL, verbose: bool = True):
    """恒速法分离粘滞摩擦 b 与库仑摩擦 fc.

    为什么扫频不够: 扫频里 dq 与 sign(dq) 高度相关(条件数 ~400), 最小二乘无法
    把 b*dq 和 fc*sign(dq) 分开 —— 惯量能辨准, 摩擦项却能差 80%。

    恒速法: 让关节稳定在若干个恒定转速, 此时 ddq≈0, 力矩只剩摩擦与重力,
    对 (omega, tau) 做直线拟合, 截距即 fc, 斜率即 b。每个速度点各测一次,
    速度是自变量而非相关量, 因此二者可分离。
    """
    m = load_bench_model(model_path)
    d = mujoco.MjData(m)
    ji = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)
    dof = m.jnt_dofadr[ji]
    aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{joint}_trq')
    truth = ground_truth(m, d, joint)
    dt = m.opt.timestep
    limited = bool(m.jnt_limited[ji])
    # 竖直轴无重力力矩; pitch 轴绕水平轴转, 有
    has_gravity = joint == 'pitch_joint'

    def run_at(w):
        """在恒速 w 下取稳态力矩. 有限位的轴走到头就传送回起点继续。"""
        mujoco.mj_resetData(m, d)
        mujoco.mj_forward(m, d)
        n_settle, n_hold = int(settle / dt), int(hold / dt)
        acc = []
        lo, hi = m.jnt_range[ji]
        for i in range(n_settle + n_hold):
            d.ctrl[aid] = np.clip(6.0 * (w - d.qvel[dof]), -8.0, 8.0)
            mujoco.mj_step(m, d)
            if limited:
                qa = m.jnt_qposadr[ji]
                if d.qpos[qa] > hi - 0.05:
                    d.qpos[qa] = lo + 0.05
                    d.qvel[dof] = w
                elif d.qpos[qa] < lo + 0.05:
                    d.qpos[qa] = hi - 0.05
                    d.qvel[dof] = w
            if i >= n_settle:
                acc.append((d.qpos[m.jnt_qposadr[ji]], d.qvel[dof],
                            d.actuator_force[aid]))
        return np.array(acc)

    # 正反转各跑一遍, 逐样本拟合并把重力项作为待估参数一起解:
    #     tau = b*w + fc*sign(w) + g_c*cos(q) + g_s*sin(q)
    # 不用"正反相减消重力"是因为有限位的轴要传送回起点, 两个方向经过的位置分布
    # 并不相同, 重力抵消不干净(实测 pitch 轴 b 误差 124%)。显式建模则无此假设。
    # 双向数据是必须的 —— 只有单向时 sign(w) 恒定, 与常数列共线, fc 不可辨。
    samples = []
    for w in speeds:
        samples.append(run_at(w))
        samples.append(run_at(-w))
    data = np.vstack(samples)
    q_s, w_s, tau_s = data[:, 0], data[:, 1], data[:, 2]
    keep = np.abs(w_s) > 0.01
    q_s, w_s, tau_s = q_s[keep], w_s[keep], tau_s[keep]

    cols = [w_s, np.sign(w_s)]
    names = ['b', 'fc']
    if has_gravity:
        cols += [np.cos(q_s), np.sin(q_s)]
        names += ['g_cos', 'g_sin']
    A = np.column_stack(cols)
    theta, *_ = np.linalg.lstsq(A, tau_s, rcond=None)
    est = dict(zip(names, theta))
    resid = tau_s - A @ theta
    r2 = 1.0 - resid.var() / tau_s.var() if tau_s.var() > 0 else float('nan')

    if verbose:
        print(f'\n=== {joint} 恒速摩擦分离 (双向, {len(speeds)} 个速度点) ===')
        print(f'    样本 {len(tau_s)}   R2={r2:.4f}   '
              f'{"含重力项" if has_gravity else "无重力项(轴竖直)"}')
        for k in names:
            tv = truth.get(k)
            if tv is not None:
                rel = abs(est[k] - tv) / max(abs(tv), 1e-9) * 100
                print(f'    {k:<6} = {est[k]:>10.5f}   真值 {tv:>9.5f}   '
                      f'{rel:>6.1f}%' if tv else
                      f'    {k:<6} = {est[k]:>10.5f}   真值 {tv:>9.5f}')
            else:
                print(f'    {k:<6} = {est[k]:>10.5f}   (重力系数)')
    return est, truth


# ---------------------------------------------------------------- 主流程
def identify_joint(joint: str, mode: str = 'chirp', duration: float = 12.0,
                   amp: float = 1.2, seed: int = 0, model_path: str = MODEL,
                   frictionloss: float | None = None, bench: bool = True,
                   verbose: bool = True):
    m = load_bench_model(model_path) if bench else mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    ji = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)
    dof = m.jnt_dofadr[ji]
    if frictionloss is not None:
        m.dof_frictionloss[dof] = frictionloss

    truth = ground_truth(m, d, joint)
    mujoco.mj_resetData(m, d)
    if not bench:
        d.qpos[2] = 0.096
    mujoco.mj_forward(m, d)

    dt = m.opt.timestep
    n = int(duration / dt)
    t = np.arange(n) * dt
    rng = np.random.default_rng(seed)
    has_limit = bool(m.jnt_limited[ji])
    if mode == 'chirp':
        if has_limit:
            # 有限位的轴必须用恒位移扫频, 否则低频段直接顶死在限位上
            span = float(np.diff(m.jnt_range[ji])[0])
            tau_cmd = chirp_shaped(t, 0.3, 12.0, truth['J'], excursion=0.35 * span / 2)
        else:
            tau_cmd = chirp(t, 0.2, 12.0, amp)
    elif mode == 'prbs':
        tau_cmd = prbs(n, amp, hold=int(0.05 / dt), rng=rng)
    elif mode == 'step':
        tau_cmd = np.where(t > 0.5, amp, 0.0)
    else:
        raise ValueError(f'未知激励 {mode}')

    q, dq, ddq, tau, cons = run_experiment(m, d, joint, tau_cmd, pin_base=not bench)
    est, r2, cond, nused = fit(q, dq, ddq, tau, constraint=cons,
                               with_gravity=(joint == 'pitch_joint'))

    if verbose:
        print(f'\n=== {joint}  激励={mode}  时长={duration}s  '
              f'{"台架(底盘焊死)" if bench else "整车自由"} ===')
        print(f'    有效样本 {nused}/{n}   R2={r2:.4f}   条件数={cond:.1f}')
        print(f"    {'参数':<6} {'辨识值':>12} {'真值':>12} {'相对误差':>10}")
        for k in ('J', 'b', 'fc'):
            tv = truth.get(k)
            ev = est.get(k, float('nan'))
            rel = abs(ev - tv) / tv * 100 if tv else float('nan')
            tag = f'{rel:>9.2f}%' if tv else '        -'
            print(f'    {k:<6} {ev:>12.5f} {tv:>12.5f} {tag}')
        if 'mgl' in est:
            print(f"    {'mgl':<6} {est['mgl']:>12.5f} {'(重力项)':>12}")
        print(f"    armature(折算转子惯量) 占 J 的 "
              f"{100 * truth['armature'] / truth['J']:.1f}%")
    return est, truth, dict(r2=r2, cond=cond, n=nused)


def main():
    ap = argparse.ArgumentParser(description='云台系统辨识')
    ap.add_argument('--joint', default='pitch_joint', choices=JOINTS)
    ap.add_argument('--mode', default='chirp', choices=('chirp', 'prbs', 'step'))
    ap.add_argument('--duration', type=float, default=12.0)
    ap.add_argument('--amp', type=float, default=1.2)
    ap.add_argument('--frictionloss', type=float, default=None,
                    help='给模型注入库仑摩擦, 用于检验辨识能否把它找回来')
    ap.add_argument('--all', action='store_true', help='三轴全跑')
    ap.add_argument('--friction', action='store_true',
                    help='额外跑恒速实验分离 b 与 fc')
    args = ap.parse_args()

    joints = JOINTS if args.all else (args.joint,)
    amps = {'big_yaw_joint': 3.0, 'small_yaw_joint': 1.0, 'pitch_joint': 1.0}
    worst = 0.0
    for j in joints:
        amp = amps.get(j, args.amp) if args.all else args.amp
        est, truth, info = identify_joint(j, args.mode, args.duration, amp,
                                          frictionloss=args.frictionloss)
        worst = max(worst, abs(est['J'] - truth['J']) / truth['J'])
        if args.friction:
            friction_sweep(j)
    print(f'\n惯量辨识最大相对误差: {worst * 100:.2f}%')
    return 0 if worst < 0.05 else 1


if __name__ == '__main__':
    raise SystemExit(main())
