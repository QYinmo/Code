from typing import Tuple, Optional
from typing import Tuple, Optional, List, Dict, Union
import math
import time
from typing import List, Literal, Optional, Tuple
from sklearn.cluster import DBSCAN
from loguru import logger
from vision.Vision_plus import debug_imshow
import cv2
import numpy as np

from FlightController.Components.LDRadar_Resolver import Point_2D

############ 参数设置##############
KERNAL_DI = 9  # 膨胀核大小
KERNAL_ER = 5  # 腐蚀核大小
HOUGH_THRESHOLD = 110
MIN_LINE_LENGTH = 200
# KERNAL_DI = 9  # 膨胀核大小
# KERNAL_ER = 5  # 腐蚀核大小
# HOUGH_THRESHOLD = 50
# MIN_LINE_LENGTH = 60
MAX_LINE_GAP = 200
#################################
kernel_di = cv2.getStructuringElement(
    cv2.MORPH_ELLIPSE, (KERNAL_DI, KERNAL_DI))
kernel_er = cv2.getStructuringElement(
    cv2.MORPH_ELLIPSE, (KERNAL_ER, KERNAL_ER))

x_dbg = 0.0
y_dbg = 0.0
yaw_dbg = 0.0


def hough_lines_to_slope_intercept(lines, output_form='both'):
    """
    将霍夫直线或HoughLinesP线段转换为指定的斜截式形式。
    Args:
        lines: cv2.HoughLines的输出 [N,1,2] (rho, theta) 或
            cv2.HoughLinesP的输出 [N,1,4] (x1, y1, x2, y2)
            或直接是已经扁平化的 [N, 4] 形状 (x1, y1, x2, y2)
            或者甚至单个线段 [x1, y1, x2, y2]
        output_form: 
            'y_kx_b' - 只返回y=kx+b形式
            'x_ky_b' - 只返回x=ky+b形式  
            'both'   - 返回两种形式（默认）
    return: 
        根据output_form返回：
        - 'both':   [{'y_kx_b': (k,b), 'x_ky_b': (k,b)}, ...]
        - 'y_kx_b': [(k,b), ...] 
        - 'x_ky_b': [(k,b), ...]
    """
    if lines is None:
        return []

    # 确保 lines 是一个可迭代的数组，即使只有一个线段
    if isinstance(lines, np.ndarray) and lines.ndim == 1 and lines.shape[0] == 4:
        # 如果传入的是单个 [x1, y1, x2, y2]
        lines_to_process = [lines]
    elif isinstance(lines, np.ndarray) and lines.ndim == 3 and lines.shape[2] == 2:
        # 如果是 HoughLines 的输出 [N, 1, 2] (rho, theta)
        lines_to_process = lines.reshape(-1, 2)
    elif isinstance(lines, np.ndarray) and lines.ndim == 3 and lines.shape[2] == 4:
        # 如果是 HoughLinesP 的输出 [N, 1, 4] (x1, y1, x2, y2)
        lines_to_process = lines.reshape(-1, 4)
    elif isinstance(lines, np.ndarray) and lines.ndim == 2 and lines.shape[1] == 4:
        # 如果是已经扁平化的 [N, 4] 形状 (x1, y1, x2, y2)
        lines_to_process = lines
    else:
        # 如果传入的是其他格式，例如 List[Tuple[float, float]]，可以直接处理
        # 或者需要更严格的错误检查
        lines_to_process = lines

    equations = []

    for line_data in lines_to_process:
        rho, theta = 0.0, 0.0

        # 判断输入线段的格式 (rho, theta) 还是 (x1, y1, x2, y2)
        if len(line_data) == 2:  # 假定是 (rho, theta)
            rho, theta = line_data
        elif len(line_data) == 4:  # 假定是 (x1, y1, x2, y2)
            x1, y1, x2, y2 = line_data
            norm = np.sqrt((y2-y1)**2 + (x2-x1)**2)
            if norm < 1e-6:  # 两点重合或距离太近，无法形成有效直线
                continue  # 跳过此线段
            A = (y2-y1) / norm
            B = (x1-x2) / norm  # 这里是 x1-x2 而非 -(x2-x1) 因为我们需要 A*x + B*y = rho 的形式
            # 计算 theta
            theta = np.arctan2(B, A)
            # 确保 theta 在 0 到 pi 之间，rho 是正值
            if theta < 0:
                theta += np.pi
            # 计算 rho
            # rho = x*cos(theta) + y*sin(theta)
            # 使用中点来计算rho更稳定
            mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
            rho = mid_x * np.cos(theta) + mid_y * np.sin(theta)
            # 如果 rho 是负数，调整 theta 使 rho 为正 (OpenCV HoughLines 约定)
            if rho < 0:
                rho = -rho
                theta += np.pi
                if theta > np.pi:
                    theta -= np.pi  # 保持在 0 到 pi
        else:
            print(f"警告: 无法识别的线段格式: {line_data}")
            continue
        a = np.cos(theta)
        b = np.sin(theta)

        # 处理y=kx+b形式
        if abs(b) > 1e-6:  # 非垂直线 (斜率不为inf)
            k_y = -a / b
            b_y = rho / b
            y_kx_b = (k_y, b_y)
        else:  # 垂直线 (x=const)
            y_kx_b = (np.inf, rho / a)  # 用inf表示斜率无穷大 (垂直线)

        # 处理x=ky+b形式
        if abs(a) > 1e-6:  # 非水平线 (斜率不为inf)
            k_x = -b / a
            b_x = rho / a
            x_ky_b = (k_x, b_x)
        else:  # 水平线 (y=const)
            x_ky_b = (np.inf, rho / b)  # 用inf表示斜率无穷大 (水平线)

        # 根据参数选择输出
        if output_form == 'y_kx_b':
            equations.append(y_kx_b)
        elif output_form == 'x_ky_b':
            equations.append(x_ky_b)
        else:
            equations.append({
                'y_kx_b': y_kx_b,
                'x_ky_b': x_ky_b
            })

    return equations


def fast_white_clusters1(image, dist_threshold=20, min_points=3):
    """
    基于距离矩阵的快速白点聚类
    Args:
        image: 输入图像
        dist_threshold: 聚类距离阈值(像素)
        min_points: 最小聚类点数
    return: 
        聚类中心列表 [(x1,y1), ...]
    """
    # 1. 二值化提取白点 (约2ms@600x400)
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    # 2. 获取坐标点 (约1ms)
    y_coords, x_coords = np.where(binary == 255)
    if len(x_coords) == 0:
        return []
    points = np.column_stack((x_coords, y_coords))

    # 3. 计算距离矩阵 (优化关键步骤，约5ms@1000点)
    diff = points[:, None, :] - points[None, :, :]  # 所有点对差值
    dist_matrix = np.sqrt((diff**2).sum(axis=2))    # 欧氏距离

    # 4. 连通区域标记
    clusters = []
    visited = np.zeros(len(points), dtype=bool)

    for i in range(len(points)):
        if not visited[i]:
            # 寻找邻近点
            neighbor_mask = (dist_matrix[i] <= dist_threshold)
            neighbors = points[neighbor_mask]

            if len(neighbors) >= min_points:
                # 计算聚类中心
                center = neighbors.mean(axis=0).astype(int)
                clusters.append(tuple(center))
                visited[neighbor_mask] = True

    return clusters


def fast_white_clusters2(image: np.ndarray, dist_threshold: float = 20, min_points: int = 3) -> List[Tuple[float, float]]:
    """
    基于 DBSCAN 的快速白点聚类
    Args:
        image: 输入图像 (灰度图或彩色图)
        dist_threshold: 聚类距离阈值(像素), 对应 DBSCAN 的 eps
        min_points: 最小聚类点数, 对应 DBSCAN 的 min_samples
    return: 
        聚类中心列表 [(x1,y1), ...]
    """
    # 1. 二值化提取白点
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    # 2. 获取白点坐标
    # 注意 np.where 返回的是 (行索引, 列索引)，对应 (y, x)
    y_coords, x_coords = np.where(binary == 255)
    if len(x_coords) == 0:
        return []  # 没有白点，直接返回空列表
    points = np.column_stack((x_coords, y_coords))
    # 3. 使用 DBSCAN 进行聚类
    # eps: 邻域半径, 即 dist_threshold
    # min_samples: 形成核心点所需的最小样本数, 即 min_points
    # metric: 距离度量方式，'euclidean' 表示欧氏距离
    # n_jobs=-1 表示使用所有可用的 CPU 核心进行并行计算，加速
    db = DBSCAN(eps=dist_threshold, min_samples=min_points,
                metric='euclidean', n_jobs=-1).fit(points)
    labels = db.labels_
    # t_end_dbscan = time.time()
    # print(f"DBSCAN 聚类耗时: {(t_end_dbscan - t_start_dbscan) * 1000:.2f} ms")
    unique_labels = set(labels)
    cluster_centers = []
    # 4. 计算每个聚类的中心点
    for label in unique_labels:
        if label == -1:
            # -1 标签代表噪声点
            continue
        class_member_mask = (labels == label)
        cluster_points = points[class_member_mask]
        center_x = np.mean(cluster_points[:, 0])
        center_y = np.mean(cluster_points[:, 1])
        cluster_centers.append(
            (float(center_x), float(center_y)))

    return cluster_centers


def fast_white_clusters3(image: np.ndarray,
                         min_area: int = 10,
                         max_area: Optional[int] = None,
                         threshold_val: int = 200) -> List[Tuple[int, int]]:
    """
    基于OpenCV轮廓检测，从图像中找到白色连通区域的中心
    Args:
        image: 输入图像 (可以是BGR三通道或灰度图)
        min_area: 最小轮廓面积
        max_area: 最大轮廓面积，如果为None，则不限制最大面积                  
        threshold_val: 二值化的阈值，高于此值的像素被认为是白色 (255)

    return:
        聚类中心列表 [(x1, y1), ...] 每个中心对应一个白色连通区域的质心
    """
    # 1. 转换为灰度图
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    # 2. 二值化提取白点
    _, binary = cv2.threshold(gray, threshold_val, 255, cv2.THRESH_BINARY)
    # 3. 查找轮廓
    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    # 4. 遍历每个轮廓，筛选并计算中心
    for contour in contours:
        # 计算轮廓的面积
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        if max_area is not None and area > max_area:
            continue
        # 计算轮廓的矩 (Moments)
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append((cx, cy))

    return centers


def get_point_line_distance_np(point, lines) -> Tuple[np.ndarray, np.ndarray]:
    """
    分别计算一个点到各条线的距离

    Args:
        point: 目标点 [x,y]
        lines: 线的两个端点 [[x1,y1,x2,y2],...]

    Returns:
        距离 / pixel, 角度(-90~90)
    """
    # point = np.asarray(point)
    # lines = np.asarray(lines)
    x1, y1, x2, y2 = lines.T

    # 计算线段长度
    line_lengths = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # 计算点到线段的投影长度
    projection_lengths = (
        (point[0] - x1) * (x2 - x1) + (point[1] - y1) * (y2 - y1)
    ) / line_lengths

    # 计算投影点坐标
    px = x1 + projection_lengths * (x2 - x1) / line_lengths
    py = y1 + projection_lengths * (y2 - y1) / line_lengths

    # 计算点到投影点的距离
    distances = np.sqrt((point[0] - px) ** 2 + (point[1] - py) ** 2)

    # 计算角度
    angles = np.degrees(np.arctan2(py - point[1], px - point[0]))

    return distances, angles


def radar_resolve_obs(
    img: np.ndarray, rel: bool = True, debug: bool = False, skip_di: bool = False, skip_er: bool = False
) -> List[Tuple[Optional[float], Optional[float], Optional[float]]]:
    """从雷达点云图像中解析障碍物，对于墙类障碍物无能为力（暂时雷达解算云图算法没法处理尺度较大挡住网的情况）

    Args:
        img: 雷达点云图像(灰度图)
        rel: 是否以飞机为原点
        debug: 显示解析结果，并显示中间调试步骤
        skip_di: 是否跳过膨胀
        skip_er: 是否跳过腐蚀

    Returns:
        障碍物的位姿列表 (x,y,yaw) / pixels/deg。
        x_out: 障碍物到后侧线的距离。
        y_out: 障碍物到右侧线的距离。
        这里的yaw通常为None，因为单点障碍物无法确定朝向。
    """
    processed_img = img.copy()
    if not skip_di:
        processed_img = cv2.dilate(processed_img, kernel_di)  # 膨胀
    if not skip_er:
        processed_img = cv2.erode(processed_img, kernel_er)  # 腐蚀
    lines_p = cv2.HoughLinesP(
        processed_img,
        1,
        np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=MIN_LINE_LENGTH,
        maxLineGap=MAX_LINE_GAP,
    )
    if debug:
        display_img = cv2.cvtColor(processed_img, cv2.COLOR_GRAY2BGR)
    else:
        display_img = None
    if lines_p is None:
        if debug:
            if display_img is not None:

                debug_imshow(display_img,
                             "Final Radar Obstacles (No Lines Detected)")
        logger.warning("[RADAR] When find obs, don't have enough line")
        return []
    img_h = img.shape[0]
    img_w = img.shape[1]
    x_img_center, y_img_center = img_w // 2, img_h // 2

    lines_flat = lines_p.reshape(-1, 4)
    # 找线
    angles = np.degrees(
        np.arctan2(lines_flat[:, 3] - lines_flat[:, 1],
                   lines_flat[:, 2] - lines_flat[:, 0])
    )
    midpoints = (lines_flat[:, :2] + lines_flat[:, 2:]) / 2

    select_right = ((angles > 45) | (angles < -45)
                    ) & (midpoints[:, 0] > x_img_center)
    select_back = ((angles > -45) & (angles < 45)
                   ) & (midpoints[:, 1] > y_img_center)
    select_left = ((angles > 45) | (angles < -45)
                   ) & (midpoints[:, 0] < x_img_center)
    select_front = ((angles > -45) & (angles < 45)
                    ) & (midpoints[:, 1] < y_img_center)

    right_lines = lines_flat[select_right]
    back_lines = lines_flat[select_back]
    left_lines = lines_flat[select_left]
    front_lines = lines_flat[select_front]

    id_right_line = None
    id_back_line = None
    id_left_line = None
    id_front_line = None
    self_x_out = 0
    self_y_out = 0
    if not (right_lines.shape[0] and back_lines.shape[0] and left_lines.shape[0] and front_lines.shape[0]):
        logger.warning("[RADAR] When find obs, don't have enough line")
        if debug:
            if display_img is not None:

                debug_imshow(display_img,
                             "Final Radar Obstacles (No Lines Detected)")
        return []
    if right_lines.shape[0] != 0:
        dists, _ = get_point_line_distance_np(
            [x_img_center, y_img_center], right_lines)
        line_index = np.argmin(dists)
        id_right_line = right_lines[line_index]
        self_y_out = dists[line_index]
    if back_lines.shape[0] != 0:
        dists, _ = get_point_line_distance_np(
            [x_img_center, y_img_center], back_lines)
        line_index = np.argmin(dists)
        id_back_line = back_lines[line_index]
        self_x_out = dists[line_index]
    if left_lines.shape[0] != 0:
        dists, _ = get_point_line_distance_np(
            [x_img_center, y_img_center], left_lines)
        line_index = np.argmin(dists)
        id_left_line = left_lines[line_index]
    if front_lines.shape[0] != 0:
        dists, _ = get_point_line_distance_np(
            [x_img_center, y_img_center], front_lines)
        line_index = np.argmin(dists)
        id_front_line = front_lines[line_index]
    if (id_right_line is None) or (id_back_line is None) or (id_left_line is None) or (id_front_line is None):
        logger.warning("[RADAR] When find obs, don't have ideal line")
        if debug:
            if display_img is not None:

                debug_imshow(display_img,
                             "Final Radar Obstacles (No Ideal Lines Detected)")
        return []
    h = img.shape[0]
    w = img.shape[1]
    y_coords, x_coords = np.indices((h, w))

    result = processed_img.copy()
    # 去网
    # 后方掩模
    if id_back_line is not None:
        back_eqs = hough_lines_to_slope_intercept(
            id_back_line, output_form='y_kx_b')
        if back_eqs and back_eqs[0][0] is not None and back_eqs[0][1] is not None:
            k_back, b_back = back_eqs[0]
            if k_back != np.inf:
                mask_back = (y_coords > k_back * x_coords +
                             b_back-30)
                result[mask_back] = 0
            else:
                pass
    # 前方掩模
    if id_front_line is not None:
        front_eqs = hough_lines_to_slope_intercept(
            id_front_line, output_form='y_kx_b')
        if front_eqs and front_eqs[0][0] is not None and front_eqs[0][1] is not None:
            k_front, b_front = front_eqs[0]
            if k_front != np.inf:
                mask_front = (y_coords < k_front *
                              x_coords + b_front+30)
                result[mask_front] = 0
            else:
                pass
    # 右侧掩模
    if id_right_line is not None:
        right_eqs = hough_lines_to_slope_intercept(
            id_right_line, output_form='x_ky_b')
        if right_eqs and right_eqs[0][0] is not None and right_eqs[0][1] is not None:
            t_right, b_right = right_eqs[0]
            if t_right > 0:
                b_right = -b_right
            if t_right != np.inf:
                mask_right = (x_coords > t_right *
                              y_coords + b_right-30)
                result[mask_right] = 0
            else:  # 水平线
                pass
    # 左侧掩模
    if id_left_line is not None:
        left_eqs = hough_lines_to_slope_intercept(
            id_left_line, output_form='x_ky_b')
        if left_eqs and left_eqs[0][0] is not None and left_eqs[0][1] is not None:
            t_left, b_left = left_eqs[0]
            if t_left > 0:
                b_left = -b_left
            if t_left != np.inf:
                mask_left = (x_coords < t_left * y_coords + b_left+30)
                result[mask_left] = 0
            else:
                pass
    # 找障碍物
    # centers = fast_white_clusters3(
    #     result, min_area=10, max_area=5000, threshold_val=100)
    # centers = fast_white_clusters1(result, dist_threshold=20, min_points=5)
    centers = fast_white_clusters2(result, dist_threshold=20, min_points=5)

    resolved_obstacles_poses = []
    resolved_obstacles_poses_rel = []
    for obs_center_x, obs_center_y in centers:
        x_obs = None
        y_obs = None
        yaw_obs = None
        if id_right_line is not None:
            dists_to_right, _ = get_point_line_distance_np(
                [obs_center_x, obs_center_y], np.array([id_right_line]))
            if dists_to_right.size > 0:
                y_obs = dists_to_right[0]
        if id_back_line is not None:
            dists_to_back, _ = get_point_line_distance_np(
                [obs_center_x, obs_center_y], np.array([id_back_line]))
            if dists_to_back.size > 0:
                x_obs = dists_to_back[0]
        resolved_obstacles_poses.append((x_obs, y_obs, yaw_obs))
        resolved_obstacles_poses_rel.append(
            (x_obs-self_x_out, y_obs-self_y_out, yaw_obs))

    # 可视化部分
    if debug:
        # 1. 原始输入图像
        # cv2.namedWindow("0. Original Input Image", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("1. Processed Image (Dilated & Eroded)", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("2. All HoughLinesP Detected (Raw)", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("3. Selected Reference Lines", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("4. Image After Masking (Potential Obstacles)", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("5. Final Radar Obstacles Resolve (All Info)", cv2.WINDOW_NORMAL)
        debug_imshow(img, "0. Original Input Image")
        # 2. 膨胀/腐蚀后的图像
        debug_imshow(processed_img, "1. Processed Image (Dilated & Eroded)")
        # 3. 原始 HoughLinesP 检测到的所有线段
        temp_all_hough_lines_img = cv2.cvtColor(
            processed_img, cv2.COLOR_GRAY2BGR)
        if lines_p is not None:
            for line in lines_p:
                x1, y1, x2, y2 = line[0]
                cv2.line(temp_all_hough_lines_img, (x1, y1),
                         (x2, y2), (0, 165, 255), 1)  # 橙色细线
        debug_imshow(temp_all_hough_lines_img, "2. All HoughLinesP Detected (Raw)"
                     )
        # 4. 筛选出的环境参考线
        temp_ref_lines_img = cv2.cvtColor(processed_img, cv2.COLOR_GRAY2BGR)
        cv2.circle(temp_ref_lines_img, (x_img_center, y_img_center),
                   3, (255, 255, 0), -1)  # 青色图像中心点
        if id_right_line is not None:
            x1, y1, x2, y2 = id_right_line.flatten()
            cv2.line(temp_ref_lines_img, (x1, y1),
                     (x2, y2), (0, 255, 0), 2)  # 绿色：右侧线
        if id_back_line is not None:
            x1, y1, x2, y2 = id_back_line.flatten()
            cv2.line(temp_ref_lines_img, (x1, y1),
                     (x2, y2), (0, 255, 255), 2)  # 黄色：后侧线
        if id_left_line is not None:
            x1, y1, x2, y2 = id_left_line.flatten()
            cv2.line(temp_ref_lines_img, (x1, y1),
                     (x2, y2), (255, 0, 0), 2)  # 蓝色：左侧线
        if id_front_line is not None:
            x1, y1, x2, y2 = id_front_line.flatten()
            cv2.line(temp_ref_lines_img, (x1, y1),
                     (x2, y2), (255, 0, 255), 2)  # 紫色：前侧线
        debug_imshow(temp_ref_lines_img, "3. Selected Reference Lines")
        # 5. 掩模后的图像
        debug_imshow(result, "4. Image After Masking (Potential Obstacles)")
        # 6. 最终可视化：所有检测到的信息
        # 绘制识别出的环境参考线
        if id_right_line is not None:
            x1, y1, x2, y2 = id_right_line.flatten()
            cv2.line(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 绿色：右侧线
        if id_back_line is not None:
            x1, y1, x2, y2 = id_back_line.flatten()
            cv2.line(display_img, (x1, y1), (x2, y2),
                     (0, 255, 255), 2)  # 黄色：后侧线
        if id_left_line is not None:
            x1, y1, x2, y2 = id_left_line.flatten()
            cv2.line(display_img, (x1, y1), (x2, y2), (255, 0, 0), 2)  # 蓝色：左侧线
        if id_front_line is not None:
            x1, y1, x2, y2 = id_front_line.flatten()
            cv2.line(display_img, (x1, y1), (x2, y2),
                     (255, 0, 255), 2)  # 紫色：前侧线
        # 绘制图像中心
        cv2.circle(display_img, (x_img_center, y_img_center),
                   3, (255, 255, 0), -1)  # 青色圆点
        # 绘制障碍物中心点及其坐标信息
        for i, (obs_center_x, obs_center_y) in enumerate(centers):
            # 绘制中心点：使用较大的红色圆点
            cv2.circle(display_img, (int(obs_center_x), int(obs_center_y)),
                       radius=8, color=(0, 0, 255), thickness=-1)
            # 获取并显示计算出的物理坐标
            if i < len(resolved_obstacles_poses):
                x_o, y_o, _ = resolved_obstacles_poses[i]
                if x_o is not None and y_o is not None:
                    text = f"Obs {i+1}: ({x_o:.1f}, {y_o:.1f})"
                    cv2.putText(
                        display_img,
                        text,
                        (int(obs_center_x) + 12, int(obs_center_y) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA
                    )
                else:
                    text = f"Obs {i+1} (No Coords)"
                    cv2.putText(
                        display_img,
                        text,
                        (int(obs_center_x) + 12, int(obs_center_y) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 165, 255),  # 橙色警告
                        1,
                        cv2.LINE_AA
                    )
        # 在图像左上角显示检测到的障碍物总数
        cv2.putText(
            display_img,
            f"Total Obstacles: {len(resolved_obstacles_poses)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),  # 红色字体
            2,
            cv2.LINE_AA
        )
        # 显示或保存最终的调试图像

        debug_imshow(display_img,
                     "5. Final Radar Obstacles Resolve (All Info)")

    if rel:
        return resolved_obstacles_poses_rel
    return resolved_obstacles_poses


def merge_similar_lines(lines_flat: np.ndarray, dist_threshold: float, angle_threshold: float) -> np.ndarray:
    """
    合并相似的线段，选择最长的一条作为代表。

    Args:
        lines_flat (np.ndarray): 原始线段数组 (N, 4)
        dist_threshold (float): 中点距离阈值 (像素)
        angle_threshold (float): 角度差阈值 (度)

    Returns:
        np.ndarray: 合并后的线段数组
    """
    if len(lines_flat) == 0:
        return lines_flat

    # 计算所有线段的中点、角度和长度
    midpoints = (lines_flat[:, :2] + lines_flat[:, 2:]) / 2
    angles = np.degrees(np.arctan2(lines_flat[:, 3] - lines_flat[:, 1],
                                   lines_flat[:, 2] - lines_flat[:, 0]))
    lengths = np.linalg.norm(lines_flat[:, 2:] - lines_flat[:, :2], axis=1)

    merged_lines = []
    is_merged = [False] * len(lines_flat)

    for i in range(len(lines_flat)):
        if is_merged[i]:
            continue

        # 将当前线段作为基准线段
        base_line = lines_flat[i]
        base_midpoint = midpoints[i]
        base_angle = angles[i]

        # 寻找所有与基准线段相似的线段
        similar_lines_indices = [i]
        for j in range(i + 1, len(lines_flat)):
            if is_merged[j]:
                continue

            current_midpoint = midpoints[j]
            current_angle = angles[j]

            dist = np.linalg.norm(base_midpoint - current_midpoint)
            angle_diff = abs(base_angle - current_angle)

            if angle_diff > 180:
                angle_diff = 360 - angle_diff

            if dist < dist_threshold and angle_diff < angle_threshold:
                similar_lines_indices.append(j)

        # --- 关键改动：从相似线段中选择最长的一条 ---
        if len(similar_lines_indices) > 0:
            longest_line_index = similar_lines_indices[0]
            max_length = lengths[longest_line_index]

            for idx in similar_lines_indices[1:]:
                if lengths[idx] > max_length:
                    max_length = lengths[idx]
                    longest_line_index = idx

            merged_lines.append(lines_flat[longest_line_index])

        # 标记所有相似的线段，避免重复处理
        for idx in similar_lines_indices:
            is_merged[idx] = True

    logger.info(
        f"[Radar] Merged {len(lines_flat)} lines into {len(merged_lines)} unique lines by choosing the longest.")
    return np.array(merged_lines)


def radar_resolve_wall(
    img: np.ndarray,
    dist_threshold: float = 50,
    mode: int = 1,
    select: int = 0,
    merge_dist_threshold: float = 20,
    merge_angle_threshold: float = 10,
    debug: bool = False,
    skip_di: bool = False,
    skip_er: bool = False
) -> List[Tuple[float, float, np.ndarray]]:
    """
    从雷达点云图像中解析墙壁，根据不同的模式找到最近的墙壁。

    Args:
        img (np.ndarray): 雷达点云图像 (灰度图)
        dist_threshold (float): 判断为“墙壁”的距离阈值 (像素)
        mode (int): 
            1: 仅考虑点到线段的垂直距离 
            2: 仅考虑线段两个端点中离图像中心较近的那个端点的距离 (欧氏距离)
            3: 融合模式，将模式1和模式2的距离相加
        select(int):
            0:不选
            1-4:右后左前
        debug (bool): 是否显示解析结果和中间调试步骤
        skip_di (bool): 是否跳过膨胀
        skip_er (bool): 是否跳过腐蚀

    Returns:
        List[Tuple[float, float, np.ndarray]]: 
            - 返回一个列表，其中每个元素是一个元组，包含一条墙壁的 (角度, 距离, 线段)。
            - 如果没有符合条件的墙壁，返回空列表。
            - 如果只有一条符合条件的墙壁，返回包含一个元组的列表。
            - 如果有多条符合条件的墙壁，返回包含最近两条墙壁信息的列表。
    """
    assert mode in {1, 2, 3}, "[Radar]:MODE ERROR - Mode must be 1, 2, or 3."
    assert select in {
        0, 1, 2, 3, 4}, "[Radar]:SELECT ERROR - Select must be 0, 1, 2, 3, or 4."

    processed_img = img.copy()

    if not skip_di:
        processed_img = cv2.dilate(processed_img, kernel_di)
        logger.debug("Image dilated.")

    if not skip_er:
        processed_img = cv2.erode(processed_img, kernel_er)
        logger.debug("Image eroded.")

    lines_p = cv2.HoughLinesP(
        processed_img,
        1,
        np.pi / 180,
        threshold=20,
        minLineLength=15,
        maxLineGap=10,
    )
    if lines_p is None:
        logger.info("[Radar] No lines detected in the image.")
        return []

    if debug:
        display_img = cv2.cvtColor(processed_img, cv2.COLOR_GRAY2BGR)
    else:
        display_img = None

    lines_flat = lines_p.reshape(-1, 4)

    # 合并相似线段
    lines_flat = merge_similar_lines(
        lines_flat, merge_dist_threshold, merge_angle_threshold)

    if lines_flat.size == 0:
        logger.info("[Radar] No lines remaining after merging.")
        return []

    img_h, img_w = img.shape[0], img.shape[1]
    x_img_center, y_img_center = img_w // 2, img_h // 2
    image_center_point = np.array([x_img_center, y_img_center])

    angles = np.degrees(
        np.arctan2(lines_flat[:, 3] - lines_flat[:, 1],
                   lines_flat[:, 2] - lines_flat[:, 0])
    )
    midpoints = (lines_flat[:, :2] + lines_flat[:, 2:]) / 2

    # 根据 select 参数筛选线段
    if select != 0:
        select_right = ((angles > 45) | (angles < -45)
                        ) & (midpoints[:, 0] > x_img_center)
        select_back = ((angles > -45) & (angles < 45)
                       ) & (midpoints[:, 1] > y_img_center)
        select_left = ((angles > 45) | (angles < -45)
                       ) & (midpoints[:, 0] < x_img_center)
        select_front = ((angles > -45) & (angles < 45)
                        ) & (midpoints[:, 1] < y_img_center)

        if select == 1:
            lines_flat = lines_flat[select_right]
        elif select == 2:
            lines_flat = lines_flat[select_back]
        elif select == 3:
            lines_flat = lines_flat[select_left]
        elif select == 4:
            lines_flat = lines_flat[select_front]

    if lines_flat.size == 0:
        logger.info("[Radar] No lines detected in the selected direction.")
        return []

    # --- 模式 1: 仅垂直距离 ---
    min_dists1, ang = get_point_line_distance_np(
        [x_img_center, y_img_center], lines_flat)

    # --- 模式 2: 仅端点距离 ---
    points1 = lines_flat[:, :2]
    points2 = lines_flat[:, 2:]
    dists1_to_center = np.linalg.norm(points1 - image_center_point, axis=1)
    dists2_to_center = np.linalg.norm(points2 - image_center_point, axis=1)
    min_dists2 = np.minimum(dists1_to_center, dists2_to_center)

    # --- 模式 3: 融合距离 ---
    min_dists3 = min_dists1 + min_dists2

    if mode == 1:
        dists = min_dists1
    elif mode == 2:
        dists = min_dists2
    else:
        dists = min_dists3

    valid_indices = np.where((dists < np.inf) & (dists < dist_threshold))[0]

    if len(valid_indices) == 0:
        logger.info(
            f"[Radar] No valid lines found within the distance threshold ({dist_threshold} px).")
        return []

    # 对有效距离进行排序
    sorted_indices = valid_indices[np.argsort(dists[valid_indices])]

    results = []

    # 提取最近的直线信息
    first_line_index = sorted_indices[0]
    id_dists1 = dists[first_line_index]
    id_line_segment1 = lines_flat[first_line_index]
    id_ang1 = ang[first_line_index]
    transformed_line_segment1 = np.array([
        -id_line_segment1[1] + 0.5 * img.shape[0],
        -id_line_segment1[0] + 0.5 * img.shape[1],
        -id_line_segment1[3] + 0.5 * img.shape[0],
        -id_line_segment1[2] + 0.5 * img.shape[1]
    ])
    results.append((id_ang1, id_dists1, transformed_line_segment1))

    logger.info(
        f"[Radar] Closest wall 1 detected: distance={id_dists1:.2f} pixels.")

    # 如果有第二条直线，提取其信息
    if len(sorted_indices) >= 2:
        second_line_index = sorted_indices[1]
        id_dists2 = dists[second_line_index]
        id_line_segment2 = lines_flat[second_line_index]
        id_ang2 = ang[second_line_index]
        transformed_line_segment2 = np.array([
            -id_line_segment2[1] + 0.5 * img.shape[0],
            -id_line_segment2[0] + 0.5 * img.shape[1],
            -id_line_segment2[3] + 0.5 * img.shape[0],
            -id_line_segment2[2] + 0.5 * img.shape[1]
        ])
        results.append((id_ang2, id_dists2, transformed_line_segment2))
        logger.info(
            f"[Radar] Closest wall 2 detected: distance={id_dists2:.2f} pixels.")

    # --- 可视化部分 ---
    if debug and display_img is not None:
        # 绘制所有合并后的线段（绿色）
        for line in lines_flat:
            x1, y1, x2, y2 = line.flatten().astype(int)
            cv2.line(display_img, (x1, y1), (x2, y2), (0, 255, 0), 1)

        # 绘制最近和次近的墙壁线段（黄色和橙色）
        for i, (_, _, seg) in enumerate(results):
            # 注意: seg是转换后的坐标，所以这里应该用transformed_line_segmentX
            original_seg = lines_flat[sorted_indices[i]].flatten().astype(int)
            x1, y1, x2, y2 = original_seg
            color = (0, 255, 255) if i == 0 else (0, 165, 255)
            thickness = 8 if i == 0 else 5
            cv2.line(display_img, (x1, y1), (x2, y2), color, thickness)

        cv2.circle(display_img, image_center_point.astype(
            int), 5, (0, 0, 255), -1)

        mode_text = {1: "Mode: Perpendicular",
                     2: "Mode: Endpoint", 3: "Mode: Combined"}[mode]
        cv2.putText(display_img, mode_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)

        if len(results) > 0:
            first_dist = results[0][1]
            cv2.putText(display_img, f"Dist1: {first_dist:.1f}", (image_center_point[0] + 10, image_center_point[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
        if len(results) > 1:
            second_dist = results[1][1]
            cv2.putText(display_img, f"Dist2: {second_dist:.1f}", (image_center_point[0] + 10, image_center_point[1] + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

        debug_imshow(
            display_img, f"Final Radar Wall (Top {len(results)} Closest)")
        cv2.waitKey(1)

    return results
