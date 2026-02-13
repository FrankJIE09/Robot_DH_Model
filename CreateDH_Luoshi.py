# 珞石机械臂 - 改进DH（MDH）模型（辨识/标定结果，仅 D 带小数）
# 使用 RevoluteMDH（改进/Modified DH）
import roboticstoolbox as rtb
from math import pi
import numpy as np
from spatialmath import SE3

# 单位：长度 mm -> m；角度 度 -> 弧度
deg = pi / 180

# 改进DH表（Alpha[deg], A[mm], D[mm], Theta/offset[deg]）
# 轴1: Alpha=0,   A=0,    D=174.522,  Theta=0
# 轴2: Alpha=-90, A=0,    D=0,        Theta=0
# 轴3: Alpha=90,  A=0,    D=314.726,  Theta=0
# 轴4: Alpha=-90, A=10,   D=0,        Theta=0
# 轴5: Alpha=90,  A=-10,  D=272.656,  Theta=0
# 轴6: Alpha=-90, A=0,    D=0,        Theta=-90
# 轴7: Alpha=-90, A=0,    D=0,        Theta=90
# 轴8: Alpha=90,  A=0,    D=96.8883,  Theta=90
#末端姿态 (RPY/xyz, rad): [ 0.01671304 -0.30055842  0.07803113]
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=174.522 / 1000, a=0, alpha=0, offset=0),
        rtb.RevoluteMDH(d=0, a=0, alpha=-90 * deg, offset=0),
        rtb.RevoluteMDH(d=314.726 / 1000, a=0, alpha=90 * deg, offset=0),
        rtb.RevoluteMDH(d=0, a=10 / 1000, alpha=-90 * deg, offset=0),
        rtb.RevoluteMDH(d=272.656 / 1000, a=-10 / 1000, alpha=90 * deg, offset=0),
        rtb.RevoluteMDH(d=0, a=0, alpha=-90 * deg, offset=-90 * deg),
        rtb.RevoluteMDH(d=0, a=0, alpha=-90 * deg, offset=90 * deg),
        rtb.RevoluteMDH(d=96.8883 / 1000, a=0, alpha=90 * deg, offset=90 * deg),
    ],
    name="Luoshi",
)

# 工具箱自带的模型摘要（角度为度、长度为 m）
print(robot)

# 初始关节角（8 轴）
init_q = np.zeros(8)

# 指定关节角（度）-> 计算并打印末端位姿
q_deg = np.array([-2.414, -68.422, -2.864, 85.239, 4.680, -33.677, -5.434,0])
q_rad = q_deg * deg
T = robot.fkine(q_rad)
print("关节 (°):", q_deg)
print("末端位姿 T (4x4):")
print(T)
print("末端位置 (m):", T.t)
print("末端姿态 (RPY/xyz, rad):", T.rpy())
print("末端姿态 (RPY/xyz, °):", np.rad2deg(T.rpy()))

# 示教模式（可选，需要时取消下一行注释）
# robot.teach(init_q)

# 可选：保存可视化
# robot.plot(init_q, movie='Luoshi.png')
