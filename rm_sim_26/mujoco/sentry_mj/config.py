"""运行期配置, 支持热重载.

参考 at_vision_simulator 的做法: 仿真运行时保存 config.toml 即刻生效, 不用重启。
辨识实验里要反复改初速、阻力系数、云台增益, 热重载省掉大量重启时间。
"""
from __future__ import annotations

import os
import tomllib
from dataclasses import dataclass, field, replace

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PATH = os.path.normpath(os.path.join(HERE, '..', 'config.toml'))


@dataclass(frozen=True)
class Projectile:
    """17mm 弹丸的送弹与摩擦轮参数.

    这里没有 muzzle_speed —— 初速由摩擦轮转速决定, 是仿真出来的结果而不是输入。
    要改初速就改 fric_speed: 理论上 v ≈ ω·r, r=0.030 m, 实际因打滑略低。
    """
    fric_speed: float = 900.0         # 摩擦轮目标转速 rad/s
    feed_speed: float = 2.0           # 拨弹推进速度 m/s
    feed_offset: float = 0.045        # 送弹点相对枪口 site 的后移量 m
    spread_deg: float = 0.15          # 送弹位置扰动
    life_s: float = 6.0               # 存活时间, 超时回收
    cooldown_s: float = 0.12          # 射击间隔
    # 弹丸过摩擦轮时的步长。轮速 900 rad/s 下 dt=1ms 每步转 51°、接触点划过
    # 27mm(比弹丸直径还大), 摩擦求解失真。触发条件见 viewer._near_friction_wheels。
    launch_dt: float = 2.0e-5
    # 弹仓
    feeder_speed: float = 12.0        # 拨弹盘转速 rad/s, 决定供弹率
    auto_refill: bool = True          # 剩余弹药少于 refill_below 时自动补满
    # 阈值按整仓容量算 —— 弹仓自然堆积容量实测 712 发
    # (tools/measure_mag_capacity.py), 不是早先手搓圆桶的那 20 发。
    refill_below: int = 40


@dataclass(frozen=True)
class Gimbal:
    """云台位置环增益. 辨识前的初值, 不是最终参数。"""
    big_yaw_kp: float = 40.0
    big_yaw_kd: float = 4.0
    small_yaw_kp: float = 8.0
    small_yaw_kd: float = 0.5
    pitch_kp: float = 6.0
    pitch_kd: float = 0.4
    scan_enabled: bool = False
    scan_yaw_speed: float = 0.4       # rad/s
    scan_pitch_range: tuple = (-0.2, 0.35)
    scan_pitch_speed: float = 0.5


@dataclass(frozen=True)
class Chassis:
    wheel_radius: float = 0.076
    mount_radius: float = 0.245
    max_wheel_torque: float = 6.0
    drive_kp: float = 1.2             # 轮速环 P


@dataclass(frozen=True)
class Config:
    projectile: Projectile = field(default_factory=Projectile)
    gimbal: Gimbal = field(default_factory=Gimbal)
    chassis: Chassis = field(default_factory=Chassis)
    path: str = DEFAULT_PATH
    _mtime: float = 0.0

    @staticmethod
    def load(path: str = DEFAULT_PATH) -> 'Config':
        if not os.path.exists(path):
            return Config(path=path)
        try:
            with open(path, 'rb') as fh:
                raw = tomllib.load(fh)
        except (OSError, tomllib.TOMLDecodeError) as exc:
            print(f'[config] 解析失败, 沿用当前参数: {exc}')
            return Config(path=path)
        return Config(
            projectile=Projectile(**raw.get('projectile', {})),
            gimbal=Gimbal(**{k: (tuple(v) if isinstance(v, list) else v)
                             for k, v in raw.get('gimbal', {}).items()}),
            chassis=Chassis(**raw.get('chassis', {})),
            path=path,
            _mtime=os.path.getmtime(path),
        )

    def reload_if_changed(self) -> 'Config':
        """文件改动则重新加载, 否则原样返回. 解析失败时保留旧值。"""
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return self
        if mtime <= self._mtime:
            return self
        new = Config.load(self.path)
        print('[config] 已重载')
        return replace(new, _mtime=mtime)
