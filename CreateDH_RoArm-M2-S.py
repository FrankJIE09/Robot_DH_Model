import roboticstoolbox as rtb
from math import pi
import numpy as np
from spatialmath import SE3

# 定义机械臂的参数
d1 = 0.02
L1, L2, L3, L4 = 0.1, 0.1, 0.1, 0.1

# 构造机械臂的DH模型
robot = rtb.DHRobot(
    [
        # 定义第一个旋转关节
        rtb.RevoluteMDH(alpha=0, a=0, offset=0, d=L1, qlim=[-np.pi, np.pi]),

        # 定义第二个旋转关节
        rtb.RevoluteMDH(alpha=pi / 2, a=0, offset=pi / 2 - np.arctan(d1 / L2), d=0, qlim=[-np.pi / 2, np.pi / 2]),

        # 定义第三个旋转关节
        rtb.RevoluteMDH(alpha=0, a=np.sqrt(d1 ** 2 + L2 ** 2), offset=-(pi / 2 - np.arctan(d1 / L2)), d=0,
                        qlim=[-np.pi / 2, np.pi / 2]),

        # 定义第四个旋转关节
        rtb.RevoluteMDH(alpha=-np.pi / 2, a=L3, offset=-np.pi / 2, d=0, qlim=[-np.deg2rad(135), np.deg2rad(135)]),
        # 定义第五个旋转关节
        rtb.RevoluteMDH(alpha=-np.pi / 2, a=0, offset=-0, d=L4, qlim=[0, 0]),
    ],
    name="RoArm-M2-S"
)

# 初始姿态的关节角度
init_T = np.array([0, 0, 0, 0,0])

# 启动示教模式，允许用户通过GUI界面手动控制机械臂
robot.teach(init_T)
# 可视化机械臂的初始姿态并保存图片
# robot.plot(init_T, movie='RoArm-M2-S.png')

# 设定机械臂的关节角度
joint_angles = [0, pi/4, pi/4, pi/4,0]

# 计算正运动学解
end_effector_pose = robot.fkine(joint_angles)

# 打印末端执行器的位置和姿态
print("末端执行器的位姿:")
print(end_effector_pose)

# 设定末端执行器的目标位置和姿态
# 例如：目标位置 (x, y, z) = (0.1, 0.1, 0.1) 和单位矩阵表示的姿态
# T = SE3(0.1, 0.1, 0.1) * SE3.RPY([0, 0, 0], order='xyz')

# 计算逆运动学解
solutions = robot.ikine_LM(end_effector_pose)

# 打印逆运动学解
if solutions.success:
    print("关节角度:", solutions.q)
else:
    print("逆运动学求解失败")
