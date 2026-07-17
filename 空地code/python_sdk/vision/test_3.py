import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor, LinearRegression

# 假设这是你提供的 output_points 函数，我在这里模拟一个
# 实际使用时，你需要从你的类中调用这个方法


def output_points(data_source: np.ndarray, scale: float = 0.1, rot_angle: int = 0) -> np.ndarray:
    """
    模拟你的 output_points 函数，输出 (2, N) 格式的点云坐标
    """
    if rot_angle == 0:
        data = data_source
    else:
        # 这里为了演示，我简化了旋转逻辑
        angle = round(rot_angle * 0.01)
        data = np.roll(data_source, angle)

    select = data != -1

    # 假设 _cos_arr 和 _sin_arr 是与数据长度相同的预计算数组
    cos_arr = np.cos(np.linspace(0, np.pi, len(data)))
    sin_arr = np.sin(np.linspace(0, np.pi, len(data)))

    points_pos = np.array(
        [data[select] * cos_arr[select], -data[select] * sin_arr[select]]) * scale

    return points_pos


def find_gap_center_with_ransac(points_2_N: np.ndarray, eps=3, min_samples=5) -> tuple | None:
    """
    使用RANSAC和DBSCAN算法从 (2, N) 格式的点云中寻找缺口中心。

    Args:
        points_2_N: 形状为 (2, N) 的点云数组，第一行为x坐标，第二行为y坐标。
        eps: DBSCAN的邻域半径参数。
        min_samples: DBSCAN的最小样本数。

    Returns:
        缺口中心的 (x, y) 坐标，如果失败则返回 None。
    """
    # --- 步骤1: 格式转换 ---
    # 将 (2, N) 格式的点云转置为 (N, 2)
    point_cloud_data = points_2_N.T
    print(f"转换后的点云数据形状: {point_cloud_data.shape}")

    if point_cloud_data.shape[0] < 2 * min_samples:
        print("点云数据太少，无法进行聚类。")
        return None

    # --- 步骤2: 使用DBSCAN进行聚类 ---
    print("正在进行点云聚类...")
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    labels = dbscan.fit_predict(point_cloud_data)

    unique_labels = set(labels)
    clusters = [point_cloud_data[labels == l]
                for l in unique_labels if l != -1]

    if len(clusters) < 2:
        print("未找到足够的墙体聚类。")
        return None

    clusters.sort(key=len, reverse=True)
    wall1_points = clusters[0]
    wall2_points = clusters[1]

    # --- 步骤3: 对每面墙进行RANSAC直线拟合 ---
    print("正在对两面墙进行直线拟合...")

    ransac1 = RANSACRegressor(LinearRegression())
    ransac1.fit(wall1_points[:, 0].reshape(-1, 1), wall1_points[:, 1])

    ransac2 = RANSACRegressor(LinearRegression())
    ransac2.fit(wall2_points[:, 0].reshape(-1, 1), wall2_points[:, 1])

    # --- 步骤4: 确定缺口位置 ---
    print("正在确定缺口端点...")

    # 找到每面墙最靠近缺口的点（x坐标的极值点）
    wall1_end_point = wall1_points[np.argmax(wall1_points[:, 0])]
    wall2_end_point = wall2_points[np.argmin(wall2_points[:, 0])]

    # --- 步骤5: 计算中心坐标 ---
    center_x = (wall1_end_point[0] + wall2_end_point[0]) / 2
    center_y = (wall1_end_point[1] + wall2_end_point[1]) / 2

    gap_center = (center_x, center_y)

    print(f"\n墙1端点坐标: {wall1_end_point}")
    print(f"墙2端点坐标: {wall2_end_point}")
    print(f"缺口中心坐标: {gap_center}")

    # --- 步骤6: 可视化结果 ---
    plt.figure(figsize=(10, 6))
    plt.scatter(point_cloud_data[:, 0], point_cloud_data[:, 1],
                c=labels, s=5, cmap='viridis', label='原始点云')
    plt.scatter(wall1_end_point[0], wall1_end_point[1],
                color='red', marker='x', s=100, linewidths=3, label='墙1端点')
    plt.scatter(wall2_end_point[0], wall2_end_point[1],
                color='blue', marker='x', s=100, linewidths=3, label='墙2端点')
    plt.scatter(gap_center[0], gap_center[1], color='cyan',
                marker='o', s=150, linewidths=3, label='缺口中心')
    plt.title("缺口中心检测结果")
    plt.xlabel("X坐标")
    plt.ylabel("Y坐标")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

    return gap_center


# --- 主程序 ---
if __name__ == '__main__':
    # 假设这是你的雷达数据
    # 这里我们模拟一个包含两堵墙和噪声的数据
    data_source_mock = np.hstack([
        np.full(100, 100),           # 墙1
        np.full(100, 100),           # 墙1
        np.full(100, -1),            # 缺口
        np.full(100, -1),            # 缺口
        np.full(100, 100),           # 墙2
        np.full(100, 100)            # 墙2
    ]) + np.random.randint(-5, 5, 600)  # 添加随机噪声

    # 调用你的函数，获取 (2, N) 格式的点云数据
    points_2_N = output_points(data_source_mock, scale=0.5, rot_angle=0)

    # 传入这个数据到我们的处理函数中
    gap_center_pos = find_gap_center_with_ransac(points_2_N)
