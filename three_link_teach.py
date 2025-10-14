# -*- coding: utf-8 -*-
"""
3连杆机械臂teach模式可视化程序
基于给定的变换序列：rot(y,theta0),trans(z,l1)rot(y,theta1),trans(z,l2)rot(y,theta2),trans(z,l3)trans(x,d)rot(y,10)
使用DH参数和robottoolbox实现正逆解求解
参考CreateDH.py的代码风格

Created by Assistant
"""

import roboticstoolbox as rtb
from math import pi
import numpy as np
from spatialmath import SE3

# 3连杆机械臂参数
L0 = 0.3  # 连杆1长度

L1 = 0.3  # 连杆1长度
L2 = 0.25 # 连杆2长度  
L3 = 0.2  # 连杆3长度
D = 0.5   # 末端偏移量

# 构造3连杆机械臂DH模型
# 根据变换序列：rot(y,theta0),trans(z,l1)rot(y,theta1),trans(z,l2)rot(y,theta2),trans(z,l3)trans(x,d)rot(y,10)
# 基于分析结果，使用方案2：第一个alpha=90度，其余alpha=0度
robot = rtb.DHRobot(
    [
        # 第一个关节：rot(y,theta0) - 绕Y轴旋转
        # 需要alpha=90度来建立正确的坐标系
        rtb.RevoluteMDH(alpha=0, a=L0, offset=0, d=0),

        rtb.RevoluteMDH(alpha=0, a=L1, offset=0, d=0),
        
        # 第二个关节：trans(z,l1)rot(y,theta1) - Z轴平移l1，然后绕Y轴旋转theta1
        # Z轴平移对应d=l1，alpha=0度
        rtb.RevoluteMDH(alpha=0, a=L2, offset=-np.pi/3, d=0),
        
        # 第三个关节：trans(z,l2)rot(y,theta2) - Z轴平移l2，然后绕Y轴旋转theta2  
        # Z轴平移对应d=l2，alpha=0度
        rtb.RevoluteMDH(alpha=0, a=L3, offset=0, d=0),

    ], 
    name="Three Link Robot"
)

# 显示机械臂信息
print("=== 3连杆机械臂DH模型 ===")
print(robot)

print("\n=== 机械臂参数 ===")
print(f"连杆1长度: {L1}m")
print(f"连杆2长度: {L2}m")
print(f"连杆3长度: {L3}m")
print(f"末端偏移: {D}m")
print(f"总工作半径: {L1 + L2 + L3}m")

# 初始关节角度
init_T = np.array([0, 0, 0, 0])

# 正运动学求解
print("\n=== 正运动学求解 ===")
print(f"初始关节角度: {np.degrees(init_T[:3])} 度")
T_end = robot.fkine(init_T)
print(f"末端执行器位置: [{T_end.t[0]:.4f}, {T_end.t[1]:.4f}, {T_end.t[2]:.4f}]")

# 启动teach模式
print("\n=== 启动Teach模式 ===")
print("在teach模式中，您可以:")
print("1. 拖动滑块调整关节角度")
print("2. 观察机械臂的实时运动")
print("3. 查看末端执行器的位置和姿态")
print("4. 关闭窗口退出teach模式")

robot.teach(init_T)  # 示教模式

print("Teach模式已结束")
