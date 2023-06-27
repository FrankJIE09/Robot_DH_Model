# 该文件是一个Python脚本，主要实现了一个六连杆机械臂的正逆运动学求解以及机械臂姿态可视化。
# 其中用到了roboticstoolbox库来定义DH模型，并调用该库中的函数进行机械臂的正逆运动学求解，最后调用plot函数可视化机械臂姿态。
# 程序中的init_T和theta均为机械臂的初始姿态和运动学参数，p为机械臂工作空间内的位置，q为机械臂的关节角度。该文件主要适用于控制六连杆机械臂运动的工程师。
# Created by JieYu

import roboticstoolbox as rtb
from math import pi
import numpy as np

L1 = 0.320
L3 = 0.975
L5 = 0.200
L6 = 0.887

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha=0, a=0, offset=0, d=0),
        rtb.RevoluteMDH(alpha=pi / 2, a=L1, offset=0, d=0),
        rtb.RevoluteMDH(alpha=0, a=L3, offset=0, d=0),
        rtb.RevoluteMDH(alpha=pi / 2, a=L5, offset=0, d=L6),
        rtb.RevoluteMDH(alpha=-pi / 2, a=0, offset=0, d=0),
        rtb.RevoluteMDH(alpha=pi / 2, a=0, offset=0, d=0)
    ], name="six link")

init_T = np.array([0, pi / 6, -pi / 6, 0, 0, 0])
robot.plot(init_T, movie='six link.png')
# robot.teach() 		#示教模式
theta = [0, pi / 6, -pi / 6, 0, 0, 0]
p = robot.fkine(theta)  # 机械臂求正解
print(p)
q = robot.ikine_LM(p)  # 机械臂求逆解
print(q)
