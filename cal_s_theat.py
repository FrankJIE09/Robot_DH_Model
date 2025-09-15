import numpy as np
import pytransform3d.transformations as pt
from scipy.spatial.transform import Rotation

# --- 原始设置 ---
# 创建一个SE(3)矩阵：绕z轴旋转90度，然后沿x轴平移1个单位
R = Rotation.from_euler('z', 90, degrees=True).as_matrix()
p = np.array([1, 0, 0])
T = np.identity(4)
T[:3, :3] = R
T[:3, 3] = p

print("输入的SE(3)矩阵 T:\n", T)

# --- 修正后的方法 ---
# 1. 从变换矩阵 T 计算指数坐标（即运动旋量/twist向量 ξ）。
# 指数坐标的定义是 ξ = S * θ，其中 S 是螺旋轴，θ 是旋转角。
exp_coords = pt.screw_matrix_from_transform_log(T)

# 2. 从指数坐标中提取旋转角 θ 和螺旋轴 S。
# 旋转角 θ 是指数坐标旋转部分（后3个元素）的范数（即模长）。
theta = np.linalg.norm(exp_coords[3:])

# 螺旋轴 S 是归一化后的指数坐标向量。
# 这里加上一个极小的数 (epsilon) 来防止当 theta 为 0 时发生除零错误。
screw_axis = exp_coords / (theta + 1e-10)

# --- 输出结果 ---
theta_deg = np.rad2deg(theta) # 将角度从弧度转换为度

print("\n--- 修正后的 pytransform3d 转换结果 ---")
print(f"指数坐标 (ξ = [v, ω]): {np.round(exp_coords, 4)}")
print(f"旋转角度 theta (rad/弧度): {theta:.4f}")
print(f"旋转角度 theta (deg/度): {theta_deg:.4f}")
print(f"螺旋轴 (S = [v̂, ω̂]): {np.round(screw_axis, 4)}")