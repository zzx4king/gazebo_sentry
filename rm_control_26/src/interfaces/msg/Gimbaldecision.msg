# Gimbaldecision.msg

# ================= 自瞄模块 ============== 
uint8 is_auto_aim              # 是否自瞄 (0=否,1=是)
uint8 auto_aim_mode            # 自瞄模式 (模式编号)

# 自瞄目标优先级数组 (Bytes6-19: 14bytes=7*2B)
Target[] targets              # 固定7元素数组

# ================= 云台控制 ============== 
uint8 is_gimbal_patrol         # 是否巡逻 (0=否,1=是)
float32 yaw_degrees            # 目标角度 
uint8 is_reached               # 到达终点 (0=未到达,1=到达)

