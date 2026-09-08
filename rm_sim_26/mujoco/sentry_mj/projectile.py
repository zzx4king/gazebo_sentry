"""17mm 弹丸发射与回收.

物理受力由 MuJoCo 自己算, 不手写弹道:
  - 重力
  - 气动阻力与升力: MJCF 里 option density=1.204 + geom fluidshape="ellipsoid",
    MuJoCo 按椭球模型算阻力、升力、马格努斯力和角阻尼, 比自己写 0.5*rho*Cd*A*v^2
    多了自旋耦合项, 正好用来看弹丸自旋对落点的影响
  - 碰撞: condim=4, 与地面/机器人正常接触

弹丸用对象池: MJCF 里预建 N 个 free body, 发射即改 qpos/qvel, 回收即挪到地下并
清零速度。这样 mjModel 结构固定, 可以直接用于 MJX 批量仿真。
"""
from __future__ import annotations

import math

import mujoco
import numpy as np


class ProjectilePool:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, prefix: str = 'proj'):
        self.m, self.d = model, data
        self.bodies, self.qadr, self.vadr, self.gids = [], [], [], []
        i = 0
        while True:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'{prefix}{i}')
            if bid < 0:
                break
            jid = model.body_jntadr[bid]
            self.bodies.append(bid)
            self.qadr.append(model.jnt_qposadr[jid])
            self.vadr.append(model.jnt_dofadr[jid])
            self.gids.append(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                               f'{prefix}{i}_geom'))
            i += 1
        # 记下原始碰撞掩码, 停放时清零、发射时恢复
        self._ctype = [int(model.geom_contype[g]) for g in self.gids]
        self._caff = [int(model.geom_conaffinity[g]) for g in self.gids]
        self.n = len(self.bodies)
        self._next = 0
        self._fired_at = np.full(self.n, -1e9)
        self._alive = np.zeros(self.n, dtype=bool)
        self._in_mag = np.zeros(self.n, dtype=bool)      # 该槽位的弹丸在弹仓里
        self._seed_of = np.full(self.n, -1, dtype=int)   # 占用的堆放点序号
        self._last_shot = -1e9
        self.muzzle_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'muzzle')
        if self.n == 0:
            raise RuntimeError('MJCF 里没有弹丸池, 检查 gen_mjcf.py 的 N_PROJECTILE')

        # 弹药账目: 实测弹仓自然容量 (tools/measure_mag_capacity.py) 远大于池子。
        # 池子只覆盖拨弹盘附近的刚体弹丸 —— 712 发全用刚体只有 0.11x 实时。
        # 剩下的记在 reserve 里, 打掉一发就从 reserve 补一发实体进来。
        self.capacity = int(self._numeric('mag_capacity', default=self.n))
        self.seeds = self._numeric_xyz('mag_seed')
        cav = self._numeric_xyz('mag_cavity')
        self.cav_lo, self.cav_hi = (cav[0], cav[1]) if len(cav) == 2 else (None, None)
        self.reserve = 0
        self.chute_losses = 0            # 从出弹口漏走、被记回储备的发数
        # 缓存云台 body id: 每帧对 90 发弹丸各做一次 mj_name2id 是字符串哈希,
        # 实测把空转从 0.96x 实时压到 0.47x。
        self.gimbal_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gimbal_link')
        # 位置索引矩阵, 一次性取出全部弹丸坐标: qpos[self._qidx] -> (n, 3)
        self._qidx = np.array([[q, q + 1, q + 2] for q in self.qadr])
        self._quatidx = np.array([[q + 3, q + 4, q + 5, q + 6] for q in self.qadr])
        self._zidx = np.array([q + 2 for q in self.qadr])
        self._vidx = np.array([[v + i for i in range(6)] for v in self.vadr])
        self._park_xyz = np.array([[0.0, 0.0, -5.0 - 0.1 * k] for k in range(self.n)])

    # ---- 模型里的自定义数值 ----
    def _numeric(self, name: str, default: float) -> float:
        i = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_NUMERIC, name)
        if i < 0:
            return default
        return float(self.m.numeric_data[self.m.numeric_adr[i]])

    def _numeric_xyz(self, name: str) -> np.ndarray:
        i = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_NUMERIC, name)
        if i < 0:
            return np.zeros((0, 3))
        a, sz = self.m.numeric_adr[i], self.m.numeric_size[i]
        return self.m.numeric_data[a:a + sz - sz % 3].reshape(-1, 3).copy()

    # ---- 内部 ----
    def _set_collidable(self, k: int, on: bool):
        """开关某发弹丸的碰撞. MuJoCo 的 plane 在碰撞上是无限大的, 停放在 z=-5
        的弹丸会被判成穿透地面 5 米, 求解器把它们以 60 m/s 垂直弹射出去 ——
        表现就是隔一阵子凭空往正上方冒出一串弹丸。停放期间必须彻底关掉碰撞。
        """
        gid = self.gids[k]
        self.m.geom_contype[gid] = self._ctype[k] if on else 0
        self.m.geom_conaffinity[gid] = self._caff[k] if on else 0
        # 注: 曾想用 body_gravcomp 冻住停放的弹丸, 但 MuJoCo 只在编译期存在非零
        # gravcomp 时才启用该计算路径, 运行时改 m.body_gravcomp 完全无效
        # (实测 qacc 仍是 -9.81)。改为在 update() 里每帧重新归位。

    def _park(self, k: int):
        """把弹丸挪到地下并冻结, 不参与可见仿真"""
        qa, va = self.qadr[k], self.vadr[k]
        self.d.qpos[qa:qa + 3] = (0.0, 0.0, -5.0 - 0.1 * k)
        self.d.qpos[qa + 3:qa + 7] = (1.0, 0.0, 0.0, 0.0)
        self.d.qvel[va:va + 6] = 0.0
        self._set_collidable(k, False)
        self._alive[k] = False
        self._in_mag[k] = False
        self._seed_of[k] = -1

    def reset(self):
        for k in range(self.n):
            self._park(k)
        self._fired_at[:] = -1e9
        self._last_shot = -1e9
        self.reserve = 0

    # ---- 对外 ----
    def feed(self, cfg, rng: np.random.Generator | None = None) -> int | None:
        """送弹: 把一发放到摩擦轮入口, 给一点推弹速度。

        真车是拨弹盘把弹丸送进两个摩擦轮之间, 由轮子摩擦加速出膛 —— 初速不是
        设定值, 而是轮速的结果。所以这里只负责"喂进去", 加速交给接触求解器。

        返回弹丸槽位; 冷却未到返回 None。
        """
        t = self.d.time
        if t - self._last_shot < cfg.cooldown_s:
            return None
        if self.remaining <= 0:
            return None                      # 打空了, 真车也是这样
        rng = rng or np.random.default_rng()

        pos = self.d.site_xpos[self.muzzle_sid].copy()
        R = self.d.site_xmat[self.muzzle_sid].reshape(3, 3)
        forward = R[:, 2]

        if cfg.spread_deg > 0:
            sig = math.radians(cfg.spread_deg)
            pos = pos + R[:, 0] * rng.normal(0, sig * 0.004) \
                      + R[:, 1] * rng.normal(0, sig * 0.004)

        # 优先取仓内离拨弹盘最近的那发 —— 弹药是从弹仓里少掉的, 不是凭空出现的。
        # 注意: 这仍然是"瞬移到摩擦轮入口", 因为拨弹盘到摩擦轮的导弹槽还没建模
        # (那段要跨 gimbal->small_yaw->pitch 三个关节)。加速过程是真实的摩擦轮
        # 接触, 供弹路径不是。
        mag = [k for k in range(self.n) if self._in_mag[k]]
        if mag:
            fpos = self.d.site_xpos[self.muzzle_sid]
            k = min(mag, key=lambda i: float(np.linalg.norm(
                self.d.qpos[self.qadr[i]:self.qadr[i] + 3] - fpos)))
            self._in_mag[k] = False
            self._seed_of[k] = -1
        else:
            k = self._next
            self._next = (self._next + 1) % self.n
            self.reserve = max(0, self.reserve - 1)
        qa, va = self.qadr[k], self.vadr[k]
        # 放在摩擦轮入口后方, 沿膛线推进
        self.d.qpos[qa:qa + 3] = pos - forward * cfg.feed_offset
        self.d.qpos[qa + 3:qa + 7] = (1.0, 0.0, 0.0, 0.0)
        self.d.qvel[va:va + 3] = forward * cfg.feed_speed
        self.d.qvel[va + 3:va + 6] = 0.0
        self._set_collidable(k, True)
        self._fired_at[k] = t
        self._alive[k] = True
        self._last_shot = t
        return k

    # ---- 弹仓 ----
    def _cavity_mask(self) -> np.ndarray:
        """逐槽位判断弹丸是否还在弹仓腔体内, 一次算完所有弹丸.

        腔体范围记在 gimbal_link 系, 随云台一起转, 所以要把弹丸位置转回该系。
        必须向量化: 每帧对 90 发各做一次 3x3 矩阵乘, numpy 的单次调用开销就把
        空转从 1.18x 实时压到 0.48x —— 步长只有 2ms, 每帧 0.5ms 的 Python 开销
        就是四分之一预算。
        """
        if self.cav_lo is None:
            return np.ones(self.n, dtype=bool)
        P = self.d.qpos[self._qidx]                       # (n, 3)
        bid = self.gimbal_bid
        if bid >= 0:
            P = (P - self.d.xpos[bid]) @ self.d.xmat[bid].reshape(3, 3)
        return (np.all(P > self.cav_lo - 0.02, axis=1)
                & np.all(P < self.cav_hi + 0.02, axis=1))

    def _seed_world(self, si: int) -> np.ndarray:
        """把 gimbal_link 系的堆放点转到世界系 —— 云台一转, 弹仓跟着转"""
        bid = self.gimbal_bid
        if bid < 0:
            return self.seeds[si]
        R = self.d.xmat[bid].reshape(3, 3)
        return self.d.xpos[bid] + R @ self.seeds[si]

    def _place_at_seed(self, k: int, si: int):
        qa, va = self.qadr[k], self.vadr[k]
        self.d.qpos[qa:qa + 3] = self._seed_world(si)
        self.d.qpos[qa + 3:qa + 7] = (1.0, 0.0, 0.0, 0.0)
        self.d.qvel[va:va + 6] = 0.0
        self._set_collidable(k, True)
        self._alive[k] = True
        self._in_mag[k] = True
        self._seed_of[k] = si
        self._fired_at[k] = 1e9          # 仓内的不按存活时间回收

    def reload(self) -> tuple[int, int]:
        """一键满弹: 装到实测的自然容量.

        实体弹丸只放 min(池子大小, 容量) 发, 位置直接用实测沉降结果里离拨弹盘最近
        的那些点 —— 它们本来就该在那儿, 没有从天而降的沉降暂态。其余记在 reserve,
        每打掉一发就实体化一发补进来, 所以从拨弹盘看到的供弹是连续的 712 发。

        返回 (实体发数, 总弹药数)。
        """
        if len(self.seeds) == 0:
            return 0, 0
        n_phys = min(self.n, len(self.seeds), self.capacity)
        for k in range(self.n):
            if k < n_phys:
                self._place_at_seed(k, k)
            else:
                self._park(k)
        self.reserve = max(0, self.capacity - n_phys)
        return n_phys, self.remaining

    def _topup(self):
        """打掉一发后从 reserve 补一发实体, 落在弹堆顶部的空位上"""
        if self.reserve <= 0 or self._alive.all():
            return                       # 先做最便宜的判断: 没空槽就直接返回
        free_slot = int(np.argmin(self._alive))
        used = {self._seed_of[k] for k in range(self.n) if self._in_mag[k]}
        cand = [i for i in range(min(self.n, len(self.seeds))) if i not in used]
        if not cand:
            return
        si = max(cand, key=lambda i: self.seeds[i][2])   # 取最高的空位, 自然落到堆上
        self._place_at_seed(free_slot, si)
        self.reserve -= 1

    @property
    def in_magazine(self) -> int:
        """仓内实体弹丸数"""
        return int(self._in_mag.sum())

    @property
    def remaining(self) -> int:
        """剩余总弹药 = 仓内实体 + 未实体化的储备"""
        return self.in_magazine + self.reserve

    def fire(self, cfg, rng: np.random.Generator | None = None) -> bool:
        """兼容旧接口: 送弹即视为发射"""
        return self.feed(cfg, rng) is not None

    def update(self, cfg):
        """回收超时或已落地的弹丸, 并把停放中的按住不动.

        停放的弹丸不参与碰撞, 但仍受重力 —— 不按住就一直自由落体, 几十秒后
        位置和速度都变得很大, 白白消耗求解并累积数值误差。

        全程向量化。这里每帧都跑, 步长只有 2ms, 逐发做 Python 属性查找和 numpy
        小切片的话开销就占掉四分之一实时预算(实测 1.18x -> 0.48x)。
        """
        t = self.d.time
        z = self.d.qpos[self._zidx]

        # 1. 仓内弹丸漏出腔体: 真车这里接的是导弹槽, 弹丸沿槽走到摩擦轮; 导弹槽
        #    跨 gimbal->small_yaw->pitch 三个关节, 还没建模, 掉出去就一路落地。
        #    按"进了导弹槽"记回储备, 弹药账目才不会凭空少(实测 5 秒漏 1~2 发)。
        leaked = self._in_mag & ~self._cavity_mask()
        n_leak = int(leaked.sum())
        if n_leak:
            self.reserve += n_leak
            self.chute_losses += n_leak

        # 2. 打出去的弹丸超时或落到地下 -> 回收
        expired = self._alive & ~self._in_mag & (
            (t - self._fired_at > cfg.life_s) | (z < -0.5))

        for k in np.flatnonzero(leaked | expired):
            self._park(int(k))

        # 3. 停放中的按住不动 (不参与碰撞但仍受重力)
        idle = ~self._alive
        if idle.any():
            self.d.qpos[self._qidx[idle]] = self._park_xyz[idle]
            self.d.qpos[self._quatidx[idle]] = (1.0, 0.0, 0.0, 0.0)
            self.d.qvel[self._vidx[idle]] = 0.0

        # 4. 补弹。放在这里而不是 feed() 里: 刚打出去的弹丸还在飞, 槽位没释放,
        #    当场补不进来; 等它回收后槽位空出, 下一帧自然补上, 弹仓维持满载。
        self._topup()

    @property
    def alive_count(self) -> int:
        return int(self._alive.sum())

    def alive_states(self):
        """返回存活弹丸的 (位置, 速度), 供落点统计使用"""
        out = []
        for k in range(self.n):
            if self._alive[k]:
                qa, va = self.qadr[k], self.vadr[k]
                out.append((self.d.qpos[qa:qa + 3].copy(), self.d.qvel[va:va + 3].copy()))
        return out
