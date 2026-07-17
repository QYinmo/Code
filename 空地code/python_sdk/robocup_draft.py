import numpy as np
import cv2
import pandas as pd
from io import StringIO
import os

# --- 1. 原始数据 (f_x, f_y, dx, dy) ---
raw_data_string = """
f_x,f_y,dx,dy
-0.87,-1,-264,112
-0.87,-0.5,-131,112
-0.87,0,-2,112
-0.87,0.5,124,112
-0.87,1,251,112
-1.37,-1.5,-304,29
-1.37,-1,-206,29
-1.37,-0.5,-106,29
-1.37,0,-2,29
-1.37,0.5,95,29
-1.37,1,194,29
-1.37,1.5,291,29
-1.87,-1.5,-249,-23
-1.87,-1,-169,-23
-1.87,-0.5,-85,-23
-1.87,0,-2,-23
-1.87,0.5,78,-23
-1.87,1,156,-23
-1.87,1.5,237,-23
-2.37,-1.5,-210,-60
-2.37,-1,-141,-60
-2.37,-0.5,-69,-60
-2.37,0,-1,-60
-2.37,0.5,67,-60
-2.37,1,131,-60
-2.37,1.5,199,-60
-2.87,-1.5,-181,-86
-2.87,-1,-125,-86
-2.87,-0.5,-63,-86
-2.87,0,-2,-86
-2.87,0.5,55,-86
-2.87,1,110,-86
-2.87,1.5,175,-86
"""


# 使用 pd.read_csv 加载数据并转换为 np.float64
df = pd.read_csv(StringIO(raw_data_string)).astype(np.float64)

# 图像坐标 (dx, dy) 作为源点 (Source Points)
src_pts = df[['dx', 'dy']].values.astype(np.float64)
# 世界坐标 (f_y, f_x) 作为目标点 (Destination Points)

dst_pts = df[['f_y', 'f_x']].values.astype(np.float64)
f_y_old = dst_pts[:, 0]
f_x_old = dst_pts[:, 1]

# 2. 计算新分量 (NumPy操作自动保留 float64 类型)
f_y_new = f_x_old
f_x_new = -f_y_old

# 3. 构建新的目标点数组 (f_y_new, f_x_new)
dst_pts = np.column_stack((f_y_new, f_x_new))
# --- 2. 计算 H' 矩阵 (最小二乘法) ---
# 方法 0 代表最小二乘法，用于最大限度拟合所有点，获得最佳物理外推性
H_prime, _ = cv2.findHomography(src_pts, dst_pts, 0)

# 确保 H_prime 是 float64 格式
H_prime = H_prime.astype(np.float64)

# --- 3. 存储矩阵到文件 (.npy) ---
H_PRIME_FILE = 'H_prime_LS_matrix_new2.npy'
np.save(H_PRIME_FILE, H_prime)

print(f"透视矩阵 H' (最小二乘) 已成功计算并保存到 '{H_PRIME_FILE}' 文件中。")
print("\n--- 计算的 H' 矩阵 ---")
print(H_prime)

# --- 4. 训练精度验证 ---
predicted_world_pts = cv2.perspectiveTransform(
    src_pts.reshape(-1, 1, 2), H_prime).squeeze()
actual_world_pts = dst_pts

rmse_fy = np.sqrt(
    np.mean((actual_world_pts[:, 0] - predicted_world_pts[:, 0])**2))
rmse_fx = np.sqrt(
    np.mean((actual_world_pts[:, 1] - predicted_world_pts[:, 1])**2))

print("\n--- 训练精度验证 (RMSE) ---")
print(f"f_x 坐标 RMSE: {rmse_fx:.10f} 米")
print(f"f_y 坐标 RMSE: {rmse_fy:.10f} 米")
