#!/usr/bin/env python3
"""修正各 link 的质量/惯量:
   1) 去掉 gimbal_link 里重复计入的 CAD Mid360 (它在 SDF 里已是独立 link)
   2) 外购件质量按实物规格修正 (CAD 中密度=1000 者为 SW 未赋材料)
输出可直接填入 model.sdf 的数值 (ROS 系, 关于各 link 质心)。
"""
import json

import numpy as np

C = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)   # v_ros = C @ v_sw


def ros_to_sw(p):
    """ROS(Z-up) -> SW(Y-up). 手工换算极易把符号写反, 统一走这个函数。"""
    return C.T @ np.asarray(p, dtype=float)


# 自检: 往返必须还原
_t = np.array([0.1, -0.2, 0.3])
assert np.allclose(C @ ros_to_sw(_t), _t), '坐标换算自检失败'


def to_ros_I(I):        # 对角张量 SW(Y-up) -> ROS(Z-up): y<->z 互换
    return np.array([I[0], I[2], I[1]])


def shift_I(I, m, d):   # 平行轴: 从质心搬到偏移 d 处
    return I + m * np.array([d[1]**2 + d[2]**2, d[0]**2 + d[2]**2, d[0]**2 + d[1]**2])


def remove_point(m_tot, c_tot, I_tot, m_p, c_p):
    """从整体中扣除一个质点(不含自身惯量), 返回剩余的 m, c, I(关于新质心)"""
    m_new = m_tot - m_p
    c_new = (m_tot * c_tot - m_p * c_p) / m_new
    I_at_new = shift_I(I_tot, m_tot, c_new - c_tot)
    I_new = I_at_new - m_p * np.array([(c_p - c_new)[1]**2 + (c_p - c_new)[2]**2,
                                       (c_p - c_new)[0]**2 + (c_p - c_new)[2]**2,
                                       (c_p - c_new)[0]**2 + (c_p - c_new)[1]**2])
    return m_new, c_new, I_new


def add_point(m_tot, c_tot, I_tot, m_p, c_p):
    m_new = m_tot + m_p
    c_new = (m_tot * c_tot + m_p * c_p) / m_new
    I_at = shift_I(I_tot, m_tot, c_new - c_tot)
    d = c_p - c_new
    return m_new, c_new, I_at + m_p * np.array([d[1]**2 + d[2]**2,
                                                d[0]**2 + d[2]**2,
                                                d[0]**2 + d[1]**2])


def fmt(label, m, c_ros, I_ros, note=''):
    print(f"{label}  mass={m:.4f}   {note}")
    print(f"   <pose>{c_ros[0]:.4f} {c_ros[1]:.4f} {c_ros[2]:.4f} 0 0 0</pose>")
    print(f"   <ixx>{I_ros[0]:.5f}</ixx><iyy>{I_ros[1]:.5f}</iyy><izz>{I_ros[2]:.5f}</izz>\n")


# ============ gimbal_link ============
# 现值(SW系, 关于自身质心) —— 由 云台10.31 减去 小yaw总成 得到
m_g = 9.078
c_g_sw = ros_to_sw(np.array([-0.0088, -0.0270, 0.1619]))
I_g_sw = np.array([0.2056, 0.1741, 0.1770])        # SW 对角 (Ixx, Iyy, Izz)

# Mid360 在 gimbal 系的位置: SDF 里是 ROS (-0.1963,-0.0880,0.2516) -> 转回 SW
p_lidar_ros = np.array([-0.1963, -0.0880, 0.2516])
p_lidar_sw = ros_to_sw(p_lidar_ros)

M_LIDAR_CAD, M_LIDAR_REAL = 0.0526, 0.265
m1, c1, I1 = remove_point(m_g, c_g_sw, I_g_sw, M_LIDAR_CAD, p_lidar_sw)
print("=== gimbal_link: 扣除重复计入的 CAD Mid360 ===")
fmt("gimbal_link", m1, C @ c1, to_ros_I(I1), f"(原 {m_g} - CAD雷达 {M_LIDAR_CAD})")
print(f"livox_lidar link 质量应改为实物值: {M_LIDAR_REAL} kg (原 0.3 为估计)\n")

# ============ small_yaw_link: GM6020 质量修正 ============
d = json.load(open('root_asm.json'))
ra = d['rootAssembly']
inst = {}
for i in ra.get('instances', []):
    inst[i['id']] = i
for sub in d.get('subAssemblies', []):
    for i in sub.get('instances', []):
        inst[i['id']] = i
T_sy = None
p_6020 = None
for occ in ra['occurrences']:
    nm = inst.get(occ['path'][-1], {}).get('name', '')
    if '云台小yaw5' in nm and len(occ['path']) == 2:
        T_sy = np.array(occ['transform']).reshape(4, 4)
for occ in ra['occurrences']:
    nm = inst.get(occ['path'][-1], {}).get('name', '')
    if 'GM6020' in nm and len(occ['path']) == 3:
        T = np.linalg.inv(T_sy) @ np.array(occ['transform']).reshape(4, 4)
        p_6020 = T[:3, 3]
        break

m_sy = 1.754
c_sy_sw = ros_to_sw(np.array([-0.0027, 0.0295, 0.1227]))   # SDF 里存的是 ROS
I_sy_sw = np.array([0.01005, 0.00437, 0.00772])            # ROS(ixx,iyy,izz)->SW: y<->z
print(f"=== small_yaw_link: GM6020 补足质量 (CAD 0.468 -> 实物 0.960) ===")
print(f"   GM6020 在小yaw系位置(SW) = {np.round(p_6020*1000,1)} mm")
m2, c2, I2 = add_point(m_sy, c_sy_sw, I_sy_sw, 0.960 - 0.468, p_6020)
fmt("small_yaw_link", m2, C @ c2, to_ros_I(I2), "(+GM6020 0.492)")

# ============ wheel: M3508 质量修正 ============
print("=== wheel_*_link: M3508 补足质量 (CAD 0.0607 -> 实物 0.365) ===")
m_w_new = 1.039 + (0.365 - 0.0607)
print(f"   单轮 mass: 1.039 -> {m_w_new:.4f} kg")
print(f"   电机基本在轮轴上, 自转惯量 izz 增量很小; 保守按 +8% 计:")
print(f"   <ixx>0.0016</ixx><iyy>0.0016</iyy><izz>0.0032</izz>\n")

print("=== 修正后各轴等效 J 预估 ===")
J_by = to_ros_I(I1)[2] + m1 * (c1[0]**2 + c1[2]**2)
# 加上 lidar / small_yaw / pitch 的贡献由 check_inertia.py 复核
print(f"   gimbal_link 自身对大yaw轴: {J_by:.5f}  (完整值见 check_inertia.py)")
