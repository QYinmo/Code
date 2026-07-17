import numpy as np
import pandas as pd
from io import StringIO
import matplotlib.pyplot as plt

# --- 1. 数据字符串 ---
# 提取您提供的 Excel 数据片段中的 x, y, p_x, p_y 列
raw_data_string = """
x,y,p_x,p_y
-1,1,56,292
-0.5,1,189,292
0,1,318,292
0.5,1,444,292
1,1,571,292
-1.5,1.5,16,209
-1,1.5,114,209
-0.5,1.5,214,209
0,1.5,318,209
0.5,1.5,415,209
1,1.5,514,209
1.5,1.5,611,209
-1.5,2,71,157
-1,2,151,157
-0.5,2,235,157
0,2,318,157
0.5,2,398,157
1,2,476,157
1.5,2,557,157
-1.5,2.5,110,120
-1,2.5,179,120
-0.5,2.5,251,120
0,2.5,319,120
0.5,2.5,387,120
1,2.5,451,120
1.5,2.5,519,120
-1.5,3,139,94
-1,3,195,94
-0.5,3,257,94
0,3,318,94
0.5,3,375,94
1,3,430,94
1.5,3,495,94
"""


df = pd.read_csv(StringIO(raw_data_string)).astype(np.float64)

# --- 2. 确定对齐中心和缩放因子 ---

# 目标中心点：(x, y) = (0, 1)
ALIGN_X = 0.0
ALIGN_Y = 1.0

# 找到中心点对应的像素坐标
center_row = df[(df['x'] == ALIGN_X) & (df['y'] == ALIGN_Y)].iloc[0]
CENTER_PX = center_row['p_x']
CENTER_PY = center_row['p_y']

# 计算一个合理的缩放因子：将世界坐标的最大跨度映射到像素坐标的最大跨度
# 跨度用于 Y (深度) 轴，因为它变化最大
world_span = df['y'].max() - df['y'].min()
image_span = df['p_y'].max() - df['p_y'].min()
SCALE_FACTOR = image_span / world_span

print(
    f"对齐中心点 (x, y): ({ALIGN_X}, {ALIGN_Y}) -> (p_x, p_y): ({CENTER_PX}, {CENTER_PY})")
print(f"缩放因子: {SCALE_FACTOR:.2f} 像素/米")

# --- 3. 归一化和缩放数据 ---

# 世界坐标：平移到中心点，然后缩放
# 注意：x 对应宽度，y 对应深度
df['X_scaled'] = (df['x'] - ALIGN_X) * SCALE_FACTOR
df['Y_scaled'] = (df['y'] - ALIGN_Y) * SCALE_FACTOR

# 像素坐标：平移到中心点
df['PX_norm'] = df['p_x'] - CENTER_PX
df['PY_norm'] = df['p_y'] - CENTER_PY

# --- 4. 可视化：单图对比 ---
plt.figure(figsize=(10, 8))
plt.title(f'归一化对比: 世界坐标 (蓝) vs. 像素坐标 (红)\n(对齐点: x=0, y=1)', fontsize=14)
plt.xlabel('X 坐标 (宽度)', fontsize=12)
plt.ylabel('Y 坐标 (深度)', fontsize=12)

# 反转 Y 轴，使近处 (y=1) 在上方，远处 (y=4) 在下方，符合图像深度习惯
plt.gca().invert_yaxis()
plt.axhline(0, color='gray', linestyle=':')
plt.axvline(0, color='gray', linestyle=':')

# --- 绘制世界坐标点 (蓝色，矩形) ---
plt.scatter(df['X_scaled'], df['Y_scaled'], color='blue',
            marker='o', s=30, zorder=5, label='世界坐标 ($x, y$) - 缩放后')

# --- 绘制像素坐标点 (红色，梯形) ---
plt.scatter(df['PX_norm'], df['PY_norm'], color='red', marker='x',
            s=30, zorder=5, label='像素坐标 ($p_x, p_y$) - 平移后')

plt.legend()
plt.grid(True, linestyle='--', alpha=0.4)
plt.show()
