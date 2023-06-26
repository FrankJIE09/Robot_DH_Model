
import roboticstoolbox as rtb
from math import pi
import numpy as np

L1=0.320
L3=0.975
L5 = 0.200
L6 = 0.887

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha=0,    a=0,    offset=0,   d=0 ),
        rtb.RevoluteMDH(alpha=pi/2, a=L1,   offset=0,   d=0 ),
        rtb.RevoluteMDH(alpha=0,    a=L3,   offset=0,   d=0 ),
        rtb.RevoluteMDH(alpha=pi/2, a=L5,   offset=0,   d=L6),
        rtb.RevoluteMDH(alpha=-pi/2,a=0,    offset=0,   d=0 ),
        rtb.RevoluteMDH(alpha=pi/2, a=0,    offset=0,   d=0 )
    ],name="six link")

init_T = np.array([0, pi/6, -pi/6, 0, 0, 0])
robot.plot(init_T, movie='six link.png')
# robot.teach() 		#示教模式
theta = [0, pi/6, -pi/6, 0, 0, 0] 
p = robot.fkine(theta)          #机械臂求正解
print(p)	
q = robot.ikine_LM(p)		#机械臂求逆解
print(q)
input()
