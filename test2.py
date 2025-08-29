import cv2
import mediapipe as mp
import numpy as np
import math
import time

# RTB-Py specific imports
import roboticstoolbox as rtb
from spatialmath import SE3 # 用于可能的 IK 或位姿计算

# --- 配置参数 (与之前类似) ---
TRACKED_ARM = 'right'
TARGET_FPS = 10
last_update_time = 0

# --- 机器人定义 (使用 RTB-Py) ---
# 你的 6-DOF 机器人的 DH 参数 (示例 - 请替换为你的实际参数)
# 格式: (d, a, alpha, offset) - 注意 RTB-Py 的 DHLink 顺序和约定
# 单位: 米, 弧度
link1 = rtb.DHLink(d=0.1, a=0, alpha=math.pi/2, offset=0)
link2 = rtb.DHLink(d=0, a=0.5, alpha=0, offset=math.pi/2) # 注意 offset 可能需要调整
link3 = rtb.DHLink(d=0, a=0, alpha=math.pi/2, offset=-math.pi/2)
link4 = rtb.DHLink(d=0.3, a=0, alpha=-math.pi/2, offset=0)
link5 = rtb.DHLink(d=0, a=0, alpha=math.pi/2, offset=0)
link6 = rtb.DHLink(d=0.1, a=0, alpha=0, offset=0)

# 创建 DHRobot 对象
robot = rtb.DHRobot(
    [link1, link2, link3, link4, link5, link6],
    name="My6DOF_Robot"
)

# 机器人关节角度限制 (示例，单位：弧度) - 需要根据你的机器人填写
# RTB-Py 在 DHRobot 对象中存储限制: robot.qlim -> shape (2, N)
robot.qlim = np.array([
    [-math.pi, math.pi],        # Joint 1 min/max
    [-math.pi/2, math.pi/2],    # Joint 2 min/max
    [-math.pi/2, math.pi/2],    # Joint 3 min/max
    [-math.pi, math.pi],        # Joint 4 min/max
    [-math.pi/2, math.pi/2],    # Joint 5 min/max
    [-math.pi, math.pi]         # Joint 6 min/max
]).T # 转置以匹配 RTB-Py 的 (2, N) 格式

print(robot)
print(f"Robot Joint Limits:\n{robot.qlim}")

# --- RTB-Py 仿真环境初始化 ---
env = None # 在主循环开始时初始化

def connect_to_simulator():
    """初始化 RTB-Py 的可视化环境"""
    global env
    try:
        # 使用 SwiftVisualizer (推荐)
        env = rtb.backends.Swift()
        env.launch(realtime=True) # 实时模式
        env.add(robot) # 将机器人添加到环境中
        print("RTB-Py Swift Visualizer launched.")
        return True
    except Exception as e:
        print(f"Failed to launch RTB-Py Swift Visualizer: {e}")
        # 备选：使用 Matplotlib (交互性较差)
        # try:
        #     env = rtb.backends.PyPlot()
        #     env.launch()
        #     env.add(robot)
        #     print("RTB-Py PyPlot Visualizer launched.")
        #     return True
        # except Exception as e2:
        #     print(f"Failed to launch RTB-Py PyPlot Visualizer: {e2}")
        #     return False
        return False


def send_joint_angles(robot_angles):
    """更新 RTB-Py 机器人对象的状态并刷新可视化"""
    if env is None:
        return
    robot.q = robot_angles # 更新机器人关节角度
    env.step() # 更新可视化环境

def check_self_collision(robot_angles):
    """
    检查自碰撞 (RTB-Py 中相对基础)。
    这里提供一个非常简化的示例，检查是否超出关节极限。
    更复杂的检查需要定义碰撞模型并检查距离。
    """
    # 1. 检查关节极限 (RTB-Py 的 robot.q 会自动裁剪，但这里显式检查)
    qlim = robot.qlim
    for i in range(robot.n):
        if not (qlim[0, i] <= robot_angles[i] <= qlim[1, i]):
            # print(f"Collision Check: Joint {i+1} limit violated.")
            return True # 超出极限视为碰撞

    # 2. 基础的链接间碰撞检测 (需要额外工作)
    #    - 你需要为每个 Link 定义碰撞几何体 (e.g., Sphere, Capsule)。
    #    - 遍历所有非相邻的 Link 对。
    #    - 计算它们之间的距离。
    #    - 如果距离小于某个阈值，则认为发生碰撞。
    #    - RTB-Py 可能没有内置的高效自碰撞检测函数，需要自行实现或查找社区方案。
    #    - 示例概念 (不完整):
    #      is_collided = robot.collided(robot_angles) # 假设有这样一个理想的函数
    #      if is_collided: return True

    # 在这个简化版本中，我们仅检查关节极限
    # 因此，这个函数主要防止因超出限制导致的“碰撞”
    return False # 假设未超出关节极限就 OK (!!!非常不安全!!!)


# --- MediaPipe 初始化 (与之前相同) ---
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

pose = mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    model_complexity=1,
    static_image_mode=False,
    enable_segmentation=False)

# --- 手臂角度计算函数 (与之前相同) ---
def calculate_vector(p1, p2):
    if p1 is None or p2 is None: return None
    return np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])

def calculate_angle_3d(v1, v2):
    if v1 is None or v2 is None or np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0: return None
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)
    dot_product = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
    return np.arccos(dot_product)


# --- 主程序 ---
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Cannot open camera")
    exit()

# 连接仿真器/可视化器
sim_connected = connect_to_simulator()
if not sim_connected:
    print("Error: Could not initialize RTB-Py environment.")
    cap.release()
    exit()

last_valid_robot_angles = robot.qz # 使用机器人的零角度作为初始状态

try:
    while cap.isOpened() and env.isopen(): # 检查可视化窗口是否关闭
        # --- 限制帧率 ---
        current_time = time.time()
        if (current_time - last_update_time) < (1.0 / TARGET_FPS):
            time.sleep(0.01) # 短暂休眠避免 CPU 占用过高
            continue
        last_update_time = current_time

        # --- 图像处理与姿态估计 ---
        success, image = cap.read()
        if not success: continue
        image.flags.writeable = False
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image_rgb)
        image.flags.writeable = True
        # image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR) # 在后面绘制前转回

        landmarks = results.pose_world_landmarks
        human_angles = {}

        if landmarks:
            # ... (提取 shoulder, elbow, wrist, hip - 与之前代码相同) ...
            lm = landmarks.landmark
            lmPose = mp_pose.PoseLandmark
            if TRACKED_ARM == 'right':
                shoulder = lm[lmPose.RIGHT_SHOULDER]; elbow = lm[lmPose.RIGHT_ELBOW]; wrist = lm[lmPose.RIGHT_WRIST]; hip = lm[lmPose.RIGHT_HIP]
            else:
                shoulder = lm[lmPose.LEFT_SHOULDER]; elbow = lm[lmPose.LEFT_ELBOW]; wrist = lm[lmPose.LEFT_WRIST]; hip = lm[lmPose.LEFT_HIP]

            # ... (计算 v_se, v_ew, v_hs - 与之前代码相同) ...
            v_se = calculate_vector(shoulder, elbow); v_ew = calculate_vector(elbow, wrist); v_hs = calculate_vector(hip, shoulder)

            # ... (计算 human_angles['elbow_flexion'], human_angles['shoulder_abduction'] 等 - 与之前代码相同) ...
            if v_se is not None and v_ew is not None:
                elbow_angle = calculate_angle_3d(v_se, -v_ew)
                if elbow_angle is not None: human_angles['elbow_flexion'] = math.pi - elbow_angle
            if v_se is not None and v_hs is not None:
                v_down_ref = -v_hs / np.linalg.norm(v_hs)
                shoulder_abd_angle = calculate_angle_3d(v_se, v_down_ref)
                if shoulder_abd_angle is not None: human_angles['shoulder_abduction'] = shoulder_abd_angle

            # --- 绘制 MediaPipe 结果 ---
            image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR) # 转回 BGR 用于绘制
            mp_drawing.draw_landmarks(
                image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        else:
            image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR) # 也要转回

        # --- 映射到机器人关节 (与之前类似，但使用 robot.qlim) ---
        target_robot_angles = list(last_valid_robot_angles) # 从上一次有效状态开始
        qlim = robot.qlim

        if 'elbow_flexion' in human_angles:
            human_elbow_min, human_elbow_max = 0.1, 2.8
            robot_j4_min, robot_j4_max = qlim[0, 3], qlim[1, 3] # 从 robot.qlim 获取
            scale = (robot_j4_max - robot_j4_min) / (human_elbow_max - human_elbow_min)
            offset = robot_j4_min - human_elbow_min * scale
            target_robot_angles[3] = np.clip(human_angles['elbow_flexion'] * scale + offset, robot_j4_min, robot_j4_max)

        if 'shoulder_abduction' in human_angles:
            human_sh_abd_min, human_sh_abd_max = 0.1, math.pi/1.9
            robot_j2_min, robot_j2_max = qlim[0, 1], qlim[1, 1] # 从 robot.qlim 获取
            scale = (robot_j2_max - robot_j2_min) / (human_sh_abd_max - human_sh_abd_min)
            offset = robot_j2_min - human_sh_abd_min * scale
            target_robot_angles[1] = np.clip(human_angles['shoulder_abduction'] * scale + offset, robot_j2_min, robot_j2_max)

        # ... (映射其他关节) ...


        # --- 碰撞检测与指令发送 (使用 RTB-Py 函数) ---
        if sim_connected:
            # 注意：RTB-Py 的 robot.q 赋值时通常会自动应用clip，但显式检查更好
            safe_angles = True
            clipped_angles = np.clip(target_robot_angles, qlim[0, :], qlim[1, :])
            if not np.allclose(target_robot_angles, clipped_angles):
                # print("Warning: Target angles clipped to joint limits.")
                target_robot_angles = clipped_angles
                # safe_angles = False # 可以选择不发送

            if safe_angles:
                # 调用 RTB-Py 的碰撞检测占位符
                if not check_self_collision(target_robot_angles):
                    # 如果不碰撞，发送指令
                    send_joint_angles(target_robot_angles)
                    last_valid_robot_angles = target_robot_angles # 更新最后有效状态
                else:
                    print("Warning: Potential self-collision detected (RTB-Py)! Command skipped.")
                    # 保持上一个状态
                    send_joint_angles(last_valid_robot_angles)

        # --- 显示图像 (与之前相同) ---
        if 'elbow_flexion' in human_angles: cv2.putText(image, f"Elbow: {math.degrees(human_angles['elbow_flexion']):.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if 'shoulder_abduction' in human_angles: cv2.putText(image, f"Shoulder Abd: {math.degrees(human_angles['shoulder_abduction']):.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))

        # --- 退出条件 ---
        if cv2.waitKey(5) & 0xFF == 27:
            break

finally:
    # --- 清理 ---
    pose.close()
    cap.release()
    cv2.destroyAllWindows()
    if env is not None and env.isopen():
        env.close()
    print("Application finished.")