# mypy: ignore-errors
from sklearn.cluster import DBSCAN
from time import perf_counter  # 更高精度计时器
import inspect
import os
import time
from typing import List, Optional, Tuple, Union
from loguru import logger
import cv2
import numpy as np
import pupil_apriltags as apriltag
from scipy import ndimage

_DEBUG = True
_DEBUG_SAVEIMG = False


def vision_debug(saveimg=False) -> None:
    """
    开启视觉模块调试功能
    启用下列三个窗口用于调试:
    Origin, Process, Result
    """
    global _DEBUG, _DEBUG_SAVEIMG
    _DEBUG = True
    _DEBUG_SAVEIMG = saveimg
    if not _DEBUG_SAVEIMG:
        x_offset = 400
        y_offset = 50
        empty_frame = np.zeros((100, 100, 3), dtype=np.uint8)
        cv2.namedWindow("Origin", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Process", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Result", cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow("Origin", 0, 0)
        cv2.moveWindow("Process", x_offset, y_offset)
        cv2.moveWindow("Result", x_offset * 2, y_offset * 2)
        cv2.imshow("Origin", empty_frame)
        cv2.imshow("Process", empty_frame)
        cv2.imshow("Result", empty_frame)
        cv2.waitKey(10)


def debug_imshow(image, name=None, idx=0):
    if not _DEBUG or image is None:
        return

    # 自动生成窗口名（如果未提供 name）
    if name is None:
        caller = inspect.stack()[1][3]  # 获取调用函数名
        name = f"{caller}_{idx}" if idx != 0 else caller

    if _DEBUG_SAVEIMG:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        name_save = f"{timestamp}_{name}.png"
        os.makedirs("debug_images", exist_ok=True)
        cv2.imwrite(f"debug_images/{name_save}.jpg", image)
    else:
        cv2.imshow(name, image)
        cv2.waitKey(1)


def find_QRcode_zbar(frame) -> Tuple[bool, float, float, str]:
    """
    使用pyzbar寻找条码
    return: 是否找到条码, x偏移值(右正), y偏移值(下正), 条码内容
    """
    from pyzbar import pyzbar

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 检测条码
    barcodes = pyzbar.decode(gray)
    if not barcodes:
        return False, 0, 0, ""

    # 获取图像尺寸
    height, width = gray.shape

    for barcode in barcodes:
        # 获取条码位置和数据
        (x, y, w, h) = barcode.rect
        try:
            data = barcode.data.decode("utf-8")
        except UnicodeDecodeError:
            data = "DecodeError: " + str(barcode.data)

        # 计算中心点和偏移量
        center_x = x + w // 2
        center_y = y + h // 2
        x_offset = center_x - width // 2
        y_offset = center_y - height // 2

        # 调试可视化
        if _DEBUG:
            debug_img = frame.copy()

            # 绘制边界框和中心点
            cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(debug_img, (center_x, center_y), 5, (0, 255, 0), -1)

            # 添加十字标记
            cv2.line(debug_img, (center_x-15, center_y),
                     (center_x+15, center_y), (0, 255, 0), 2)
            cv2.line(debug_img, (center_x, center_y-15),
                     (center_x, center_y+15), (0, 255, 0), 2)

            # 添加信息文本
            info_text = f"{data} | Offset: ({x_offset:.1f}, {y_offset:.1f})"
            cv2.putText(debug_img, info_text, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            debug_imshow(debug_img, "QRcode Detection")

        return True, x_offset, y_offset, data

    return False, 0, 0, ""


def black_line(image, type: int = 1, theta_threshold=0.25) -> Tuple[bool, float, float, float]:
    """
    寻找画面中的黑线并返回数据
    type: 0:横线 1:竖线
    theta_threshold: 角度容许误差(不能超过45度)
    return: 是否查找到黑线, x偏移值(右正), y偏移值(下正), 弧度偏移值(顺时针正)
    """
    ######### 参数设置 #########
    LOWER = np.array([0, 60, 0])
    UPPER = np.array([150, 255, 75])
    HOUGH_THRESHOLD = 200
    ###########################
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, LOWER, UPPER)
    if _DEBUG:
        debug_imshow(mask)

    target_theta = 0 if type == 1 else np.pi / 2
    # lines = cv2.HoughLines(mask, 1, np.pi/180, threshold=400, max_theta=0.1)
    if type == 0:  # 横线
        lines = cv2.HoughLines(
            mask,
            1,
            np.pi / 180,
            threshold=HOUGH_THRESHOLD,
            min_theta=target_theta - theta_threshold,
            max_theta=target_theta + theta_threshold,
        )
    else:  # 竖线
        lines = cv2.HoughLines(
            mask,
            1,
            np.pi / 180,
            threshold=HOUGH_THRESHOLD,
            max_theta=theta_threshold,
        )
        lines2 = cv2.HoughLines(
            mask,
            1,
            np.pi / 180,
            threshold=HOUGH_THRESHOLD,
            min_theta=np.pi - theta_threshold,
        )
        if lines is not None and lines2 is not None:
            lines = np.concatenate((lines, lines2))
        elif lines is None and lines2 is not None:
            lines = lines2

    if lines is not None:
        for line in lines:
            r, theta = line[0]
            x0 = r * np.cos(theta)
            y0 = r * np.sin(theta)
            x1 = int(x0 - 1000 * np.sin(theta))
            y1 = int(y0 + 1000 * np.cos(theta))
            x2 = int(x0 + 1000 * np.sin(theta))
            y2 = int(y0 - 1000 * np.cos(theta))
            if _DEBUG:
                cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                debug_imshow(image)
            x = abs((x1 + x2) / 2)
            y = abs((y1 + y2) / 2)
            size = image.shape
            x_offset = x - size[1] / 2
            y_offset = y - size[0] / 2
            if theta > np.pi / 2 and type == 1:
                t_offset = theta - target_theta - np.pi
            else:
                t_offset = theta - target_theta
            return True, x_offset, y_offset, t_offset
    return False, 0, 0, 0


def find_yellow_code(image) -> Tuple[bool, float, float]:
    """
    寻找黄色条码
    return: 是否查找到黄色条码, x偏移值(右正), y偏移值(下正)
    """
    ######### 参数设置 #########
    # LOWER = np.array([20, 40, 100])
    # UPPER = np.array([60, 150, 255])
    LOWER = np.array([0, 78, 42])
    UPPER = np.array([56, 255, 200])
    ###########################
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 根据闸值构建掩模
    mask = cv2.inRange(hsv, LOWER, UPPER)
    # 进行开运算和闭运算
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
    closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=4)
    # 找出边界
    conts, hier = cv2.findContours(
        closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if _DEBUG:
        cv2.drawContours(image, conts, -1, (0, 255, 0), 3)  # 画出边框
        debug_imshow(image)
    if conts:
        max_area = 0.0
        max_index = 0
        for n, cnts in enumerate(conts):
            area = cv2.contourArea(cnts)
            if area > max_area:
                max_area = area
                max_index = n
        # 设定面积闸值，排除黄色小噪点影响
        cnts = conts[max_index]
        area = max_area
        if area > 800:
            M = cv2.moments(cnts)
            cx = int(M["m10"] / (M["m00"]))
            cy = int(M["m01"] / (M["m00"]))
            if _DEBUG:
                cv2.circle(image, (cx, cy), 8, (0, 0, 255), thickness=-1)
                debug_imshow(image, 1)
            size = image.shape
            return True, cx - size[1] / 2, cy - size[0] / 2
    return False, 0, 0


glob_detector = None


def find_laser_point(img) -> Tuple[bool, float, float]:
    """
    寻找激光点
    return: 是否查找到激光点, x偏移值(右正), y偏移值(下正)
    """
    global glob_detector
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # img = cv2.inRange(img, 230, 255)  # 二值化
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    RED_UPPER = np.array([10, 255, 255])
    RED_LOWER = np.array([0, 43, 46])
    RED_UPPER2 = np.array([180, 255, 255])
    RED_LOWER2 = np.array([156, 43, 46])
    mask = cv2.inRange(hsv, RED_LOWER, RED_UPPER) + \
        cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    # 把红色的区域变成白色，其他区域变成黑色
    img = cv2.bitwise_and(img, img, mask=mask)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 图像先开运算，后闭运算
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    # # 图像膨胀
    # kernel = np.ones((5, 5), np.uint8)  # 定义卷积核
    # img = cv2.dilate(img, kernel)
    if _DEBUG:
        debug_imshow(img, "Process")
    if glob_detector is None:
        # 设置Blob检测参数
        params = cv2.SimpleBlobDetector_Params()  # type: ignore
        # 设置颜色
        params.blobColor = 255
        # 设置闸值
        params.minThreshold = 1
        params.maxThreshold = 20
        # 设置面积
        params.filterByArea = True
        params.minArea = 10
        params.maxArea = 500
        # 设置圆性
        params.filterByCircularity = True
        params.minCircularity = 0.8
        # 设置惯量比
        params.filterByInertia = True
        params.minInertiaRatio = 0.2

        # detector = cv2.SimpleBlobDetector(params) # opencv<4.0
        glob_detector = cv2.SimpleBlobDetector_create(params)  # type: ignore

    # Blob检测
    keypoints = glob_detector.detect(img)

    if _DEBUG:
        img_with_keypoints = cv2.drawKeypoints(
            img,
            keypoints,
            np.array([]),
            (0, 255, 0),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )
        debug_imshow(img_with_keypoints, "Process2")
    if keypoints:
        largest_keypoint = max(keypoints, key=lambda kp: kp.size)
        x = largest_keypoint.pt[0]
        y = largest_keypoint.pt[1]
        cx = int(x)
        cy = int(y)
        img_h, img_w = img.shape[:2]
        return True, cx - img_w / 2, cy - img_h / 2

    return False, 0, 0


def find_red_area(img, min_area) -> Tuple[bool, float, float, float]:
    """
    Return:
        统一返回形式
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    RED_UPPER = np.array([8, 255, 255])
    RED_LOWER = np.array([0, 109, 80])
    RED_UPPER2 = np.array([180, 255, 255])
    RED_LOWER2 = np.array([156, 109, 80])
    mask = cv2.inRange(hsv, RED_LOWER, RED_UPPER) + \
        cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    # 把红色的区域变成白色，其他区域变成黑色
    img = cv2.bitwise_and(img, img, mask=mask)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 图像先开运算，后闭运算
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(
        img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= min_area]
    areas = [cv2.contourArea(cnt) for cnt in contours]
    sorted_contours = sorted(zip(areas, contours),
                             key=lambda x: x[0], reverse=True)
    filtered_contours = [cnt for area,
                         cnt in sorted_contours if area >= min_area]
    filtered_areas = [area for area,
                      cnt in sorted_contours if area >= min_area]
    if _DEBUG:
        result_image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(result_image, filtered_contours, -1, (0, 255, 0), 2)
        debug_imshow(result_image)
    for cnt, area in zip(filtered_contours, filtered_areas):
        # 计算区域的矩
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            # 计算中点坐标
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            # 在结果图像上绘制中点qqq
            if _DEBUG:
                cv2.circle(result_image, (center_x, center_y),
                           5, (0, 0, 255), -1)
                debug_imshow(result_image, "Process2")
            return True, center_x - img.shape[1] / 2, center_y - img.shape[0] / 2, area
    return False, 0, 0, 0


def find_anycolor_area(img, color_upper: np.array, color_lower: np.array, min_area: int, mor: bool = True) -> Tuple[bool, float, float, float]:
    debug_imshow(img, "Origin")
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    COLOR_UPPER = color_upper
    COLOR_LOWER = color_lower
    mask = cv2.inRange(hsv, COLOR_LOWER, COLOR_UPPER)
    # 把目标色的区域变成白色，其他区域变成黑色
    img = cv2.bitwise_and(img, img, mask=mask)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 图像先开运算，后闭运算
    if mor:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(
        img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= min_area]
    areas = [cv2.contourArea(cnt) for cnt in contours]
    sorted_contours = sorted(zip(areas, contours),
                             key=lambda x: x[0], reverse=True)
    filtered_contours = [cnt for area,
                         cnt in sorted_contours if area >= min_area]
    filtered_areas = [area for area,
                      cnt in sorted_contours if area >= min_area]
    if _DEBUG:
        result_image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(result_image, filtered_contours, -1, (0, 255, 0), 2)
        debug_imshow(result_image)
    for cnt, area in zip(filtered_contours, filtered_areas):
        # 计算区域的矩
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            # 计算中点坐标
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            # 在结果图像上绘制中点qqq
            if _DEBUG:
                cv2.circle(result_image, (center_x, center_y),
                           5, (0, 0, 255), -1)
                debug_imshow(result_image, 2)
            return True, center_x - img.shape[1] / 2, center_y - img.shape[0] / 2, area
    return False, 0, 0, 0


def pass_filter(img, kernel_size=3) -> np.ndarray:
    """
    高/低通滤波器
    kernel_size: 3 / 5 / g (Gaussian)
    """
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if kernel_size == 3:
        kernel_3 = np.array([[-1, -1, -1], [-1, 8, -1], [-1, -1, -1]])
        return ndimage.convolve(img, kernel_3)
    elif kernel_size == 5:
        kernel_5 = np.array(
            [
                [-1, -1, -1, -1, -1],
                [-1, 1, 2, 1, -1],
                [-1, 2, 4, 2, -1],
                [-1, 1, 2, 1, -1],
                [-1, -1, -1, -1, -1],
            ]
        )
        return ndimage.convolve(img, kernel_5)
    elif kernel_size == "g":
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        g_hpf = img - blurred
        return g_hpf
    else:
        return img


def find_QRcode_contour(frame) -> Tuple[bool, float, float]:
    """
    基于形态学轮廓寻找条码
    return: 是否找到条码, x偏移值(右正), y偏移值(下正)
    """
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换成灰度图

    # 提取图像梯度提取二维码区域
    gradX = cv2.Sobel(image, ddepth=cv2.CV_32F, dx=1,
                      dy=0, ksize=-1)  # type: ignore
    gradY = cv2.Sobel(image, ddepth=cv2.CV_32F, dx=0,
                      dy=1, ksize=-1)  # type: ignore
    gradient = cv2.subtract(gradX, gradY)
    gradient = cv2.convertScaleAbs(gradient)
    # 去噪并提取兴趣区域
    blurred = cv2.blur(gradient, (9, 9))
    (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)

    # construct a closing kernel and apply it to the thresholded image
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # 进行开运算和闭运算
    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=4)
    if _DEBUG:
        debug_imshow(closed)
    # 处理轮廓，找出最大轮廓
    cnts, _ = cv2.findContours(
        closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if cnts != []:
        size = image.shape
        c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
        # compute the rotated bounding box of the largest contour
        rect = cv2.minAreaRect(c)
        box = np.intp(cv2.boxPoints(rect))
        # 找出中心点坐标
        M = cv2.moments(c)
        cx = int(M["m10"] / (M["m00"] + 0.0001))
        cy = int(M["m01"] / (M["m00"] + 0.0001))
        x_offset = cx - size[1] / 2
        y_offset = cy - size[0] / 2
        # 做出轮廓和中心坐标点
        if _DEBUG:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            cv2.circle(image, (cx, cy), 2, (0, 255, 0), 8)  # 做出中心坐标
            cv2.drawContours(image, [box], -1, (0, 255, 0), 3)
            debug_imshow(image, 1)
        return True, x_offset, y_offset
    else:
        return False, 0, 0


def rescale_image(image, scale: float, fast: bool = False) -> np.ndarray:
    """
    调整图像大小
    image: 原图像
    scale: 缩放比例(不要超过1,那没意义)
    fast: 是否使用快速算法
    return: 缩放后的图像
    """
    if scale < 1:
        inter_mathod = cv2.INTER_AREA if not fast else cv2.INTER_NEAREST
    else:
        inter_mathod = cv2.INTER_LINEAR
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=inter_mathod)


def rescale_aspect_ratio(img, width: int, height: int) -> np.ndarray:
    """
    将图片缩放到指定宽高(保持长宽比)
    """
    h, w, _ = img.shape
    if h / w > height / width:
        img = cv2.resize(img, (height * w // h, height))
    else:
        img = cv2.resize(img, (width, width * h // w))
    return img


def get_ROI(
    img,
    ROI: Tuple[Union[int, float], ...],
) -> np.ndarray:
    """
    获取兴趣区
    ROI: 若<=1则视为相对图像尺寸的比例值
    """
    x, y, w, h = ROI
    if (x + y + w + h) <= 4:
        x = int(x * img.shape[1])
        y = int(y * img.shape[0])
        w = int(w * img.shape[1])
        h = int(h * img.shape[0])
    return img[int(y): int(y + h), int(x): int(x + w)]


class HSV(object):
    """
    常用色值HSV边界
    """

    RED_UPPER = np.array([10, 255, 255])
    RED_LOWER = np.array([0, 43, 46])
    RED_UPPER2 = np.array([180, 255, 255])
    RED_LOWER2 = np.array([156, 43, 46])
    YELLOW_UPPER = np.array([34, 255, 255])
    YELLOW_LOWER = np.array([26, 43, 46])
    GREEN_UPPER = np.array([77, 255, 255])
    GREEN_LOWER = np.array([35, 43, 46])
    BLUE_UPPER = np.array([124, 255, 255])
    BLUE_LOWER = np.array([100, 43, 46])
    ORANGE_UPPER = np.array([25, 255, 255])
    ORANGE_LOWER = np.array([11, 43, 46])
    CYAN_UPPER = np.array([99, 255, 255])
    CYAN_LOWER = np.array([78, 43, 46])
    PURPLE_UPPER = np.array([155, 255, 255])
    PURPLE_LOWER = np.array([125, 43, 46])
    BLACK_UPPER = np.array([180, 255, 46])
    BLACK_LOWER = np.array([0, 0, 0])
    GRAY_UPPER = np.array([180, 43, 220])
    GRAY_LOWER = np.array([0, 0, 46])
    WHITE_UPPER = np.array([180, 30, 255])
    WHITE_LOWER = np.array([0, 0, 221])


def color_recognition(img: np.ndarray, threshold: float = 0.4) -> Union[Tuple[str, Tuple[int, int, int, int]], None]:
    """
    颜色识别 (红绿蓝), 并返回识别到的颜色及其最大连通区域的坐标。

    Args:
        img (np.ndarray): 输入图像 (BGR 格式)。
        threshold (float): 颜色像素占比阈值，大于该阈值则认为是该颜色。

    Returns:
        统一返回形式
    """
    if img is None or img.size == 0:
        return None
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask_r1 = cv2.inRange(hsv, HSV.RED_LOWER, HSV.RED_UPPER)
    mask_r2 = cv2.inRange(hsv, HSV.RED_LOWER2, HSV.RED_UPPER2)
    mask_r = cv2.bitwise_or(mask_r1, mask_r2)

    mask_g = cv2.inRange(hsv, HSV.GREEN_LOWER, HSV.GREEN_UPPER)
    mask_b = cv2.inRange(hsv, HSV.BLUE_LOWER, HSV.BLUE_UPPER)
    red_count = cv2.countNonZero(mask_r)
    green_count = cv2.countNonZero(mask_g)
    blue_count = cv2.countNonZero(mask_b)
    max_count = max(red_count, green_count, blue_count)
    tho = int(img.shape[0] * img.shape[1] * threshold)
    if max_count == red_count and red_count > tho:
        best_color_name = "red"
        best_color_mask = mask_r
    elif max_count == green_count and green_count > tho:
        best_color_name = "green"
        best_color_mask = mask_g
    elif max_count == blue_count and blue_count > tho:
        best_color_name = "blue"
        best_color_mask = mask_b
    else:
        best_color_name = None
        best_color_mask = None

    if best_color_name is not None and best_color_mask is not None:
        contours, _ = cv2.findContours(
            best_color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x_bbox, y_bbox, w_bbox, h_bbox = cv2.boundingRect(largest_contour)
            center_x_region = x_bbox + w_bbox / 2
            center_y_region = y_bbox + h_bbox / 2
            x_img_center = img.shape[1] / 2
            y_img_center = img.shape[0] / 2
            dx = center_x_region - x_img_center
            dy = center_y_region - y_img_center
            if _DEBUG:
                display_img = img.copy()
                cv2.drawContours(
                    display_img, [largest_contour], -1, (0, 255, 255), 2)
                cv2.rectangle(display_img, (x_bbox, y_bbox), (x_bbox +
                                                              w_bbox, y_bbox + h_bbox), (0, 255, 0), 2)
                cv2.circle(display_img, (int(x_img_center), int(
                    y_img_center)), 5, (255, 0, 255), -1)  # 紫色圆点
                cv2.circle(display_img, (int(center_x_region), int(
                    center_y_region)), 5, (0, 0, 255), -1)  # 红色圆点
                info_text = f"Color: {best_color_name}"
                offset_text = f"Offset: X={dx:.1f}, Y={dy:.1f}"
                cv2.putText(display_img, info_text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(display_img, offset_text, (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                debug_imshow(display_img, "Process")
            return True, dx, dy, best_color_name
    return False, 0, 0, None


def shape_recognition(image, LOWER, UPPER):
    """
    形状识别(圆、矩形、三角形)
    LOWER, UPPER: HSV边界
    return: 识别类型文本, 无法识别为unknown
    """
    shapes = {}
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER, UPPER)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 3))
    closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=5)
    if _DEBUG:
        debug_imshow(mask)
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        for cnt in range(len(contours)):
            if _DEBUG:
                cv2.drawContours(image, contours, cnt, (0, 255, 0), 2)
            epsilon = 0.01 * cv2.arcLength(contours[cnt], True)
            approx = cv2.approxPolyDP(contours[cnt], epsilon, True)
            corners = len(approx)
            if corners == 3:
                shapes["triangle"] = shapes.get("triangle", 0) + 1
            if corners == 4:
                shapes["rectangle"] = shapes.get("rectangle", 0) + 1
            if corners >= 10:
                shapes["circle"] = shapes.get("circle", 0) + 1
    if _DEBUG:
        debug_imshow(image, 1)
    if len(shapes) == 0:
        return "unknown"
    max_shape = max(shapes, key=shapes.get)
    return max_shape


def hsv_checker(img, lower, upper, threshold=0.4) -> bool:
    """
    计算hsv图像中目标色值占比是否超过阈值
    lower, upper: hsv色值范围
    threshold: 阈值
    return: 超过阈值
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    if _DEBUG:
        debug_imshow(mask)
    return cv2.countNonZero(mask) / (img.shape[0] * img.shape[1]) > threshold


def dp_outline_calc(frame) -> float:
    """
    D-P算法轮廓面积计算
    return: 最大轮廓面积, 未找到时返回0
    """
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.GaussianBlur(gray_frame, (19, 19), 0)  # 高斯滤波进行平滑处理
    # 处理图像轮廓
    ret, thresh = cv2.threshold(gray_frame, 127, 255, 0)
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return 0
    contours = list(contours)
    contours.sort(key=len, reverse=True)  # 进行排序，寻找极大轮廓
    cnt = contours[0]
    # 进行轮廓近似
    epsilon = 0.000001 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    area = cv2.contourArea(approx)
    if _DEBUG:
        frame = cv2.drawContours(frame, approx, -1, (255, 0, 0), 3)
        debug_imshow(frame)
    return area


def FLANN_match(train_img, frame) -> Tuple[int, Tuple[float, float]]:
    """
    FLANN单应性特征匹配, 最小值匹配
    train_img: 目标查询图像
    frame: 待匹配图像
    return: 匹配点数量, 匹配中点坐标
    """
    ######### 参数设置 #########
    MIN_MATCH_COUNT = 10  # 最小匹配点数量
    FLANN_INDEX_KDTREE = 0  # FLANN索引类型
    ###########################
    train_img = train_img.copy()
    frame = frame.copy()
    sift = cv2.SIFT_create()  # type: ignore
    kp1, des1 = sift.detectAndCompute(train_img, None)
    kp2, des2 = sift.detectAndCompute(frame, None)
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    # matches = flann.knnMatch(des1, des2, k=2) # BUG:数据类型转换错误
    matches = flann.knnMatch(np.asarray(
        des1, np.float32), np.asarray(des2, np.float32), k=2)
    # 最小匹配选择
    good = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good.append(m)

    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]
                             ).reshape(-1, 1, 2)  # type: ignore
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]
                             ).reshape(-1, 1, 2)  # type: ignore

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        h, w = train_img.shape[:2]
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1],
                         [w - 1, 0]]).reshape(-1, 1, 2)  # type: ignore
        dst = np.int32(cv2.perspectiveTransform(pts, M))
        p1, p2, p3, p4 = dst.reshape(4, 2)
        center_point = (p1 + p2 + p3 + p4) / 4
        # 透视关系
        if _DEBUG:
            matchesMask = mask.ravel().tolist()
            draw_params = dict(
                matchColor=(0, 255, 0),  # draw matches in green color
                singlePointColor=None,
                matchesMask=matchesMask,  # draw only inliers
                flags=2,
            )
            img3 = cv2.drawMatches(train_img, kp1, frame,
                                   kp2, good, None, **draw_params)
            debug_imshow(img3)
            # 目标匹配
            frame = cv2.polylines(frame, [dst], True, 255, 3, cv2.LINE_AA)
            debug_imshow(frame, 1)
        ###############
        return len(good), center_point
    else:
        return 0, (0, 0)


def contours_match(train_img, frame) -> float:
    """
    轮廓匹配
    train_img: 查询图片
    frame: 待匹配图片
    return: 匹配度(越小越匹配)
    """
    # 高斯滤波降噪
    frame_p = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    train_p = cv2.cvtColor(train_img, cv2.COLOR_BGR2GRAY)
    frame_p = cv2.GaussianBlur(frame_p, (5, 5), 0)
    train_p = cv2.GaussianBlur(train_p, (19, 19), 0)
    # 处理轮廓
    ret, thresh = cv2.threshold(frame_p, 127, 255, 0)
    ret, thresh2 = cv2.threshold(train_p, 127, 255, 0)
    contours1, hierarchy = cv2.findContours(thresh, 2, 1)
    contours2, hierarchy = cv2.findContours(thresh2, 2, 1)
    if len(contours1) == 0 or len(contours2) == 0:
        return 1.0
    cnt1 = contours1[0]
    cnt2 = contours2[0]
    # 计算匹配度
    matching_value = cv2.matchShapes(cnt1, cnt2, 1, 0.0)
    if _DEBUG:
        frame = cv2.drawContours(frame, contours1, -1, (255, 0, 0), 3)
        debug_imshow(frame)
    return matching_value


class Meanshift(object):
    ######### 参数设置 #########
    LOWER = np.array((99.0, 90.0, 102.0))
    UPPER = np.array((132.0, 212.0, 157.0))
    LOW_PASS_RATIO = 1
    TERM_ITER = 10  # 终止条件: 迭代次数
    TERM_MOVE = 1  # 终止条件: 移动距离

    ###########################
    def __init__(self, init_ROI: Tuple[Union[int, float]]):
        """
        均值漂移目标跟踪
        init_ROI: 初始兴趣区域, (x, y, w, h) (角点和宽高), 传入小于1的值时, 自动计算比例
        """
        self.init_ROI = init_ROI
        self.inited = False

    def _init_local(self, frame):
        self.img_shape = frame.shape
        self.reset_roi(self.init_ROI)
        # c, r, w, h = ROI
        # roi = frame[r : r + h, c : c + w]
        hsv_roi = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 创建包含具有HSV的所有像素的掩码
        mask = cv2.inRange(hsv_roi, self.LOWER, self.UPPER)
        # 计算直方图,参数为 图片(可多)，通道数，蒙板区域，直方图长度，范围
        self.roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
        cv2.normalize(self.roi_hist, self.roi_hist,
                      0, 255, cv2.NORM_MINMAX)  # 归一化
        # 设置终止条件，迭代10次或者至少移动1次
        self.term_crit = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            self.TERM_ITER,
            self.TERM_MOVE,
        )

    def update(self, frame) -> Tuple[float, float]:
        """
        更新目标
        frame: 当前帧
        return: 目标x偏移, 目标y偏移
        """
        if not self.inited:
            self._init_local(frame)
            self.inited = True
        # 处理训练图像
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 直方图反向投影，求得每个像素点的概率
        dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0, 180], 1)
        # 调用meanShift算法在dst中寻找目标窗口，找到后返回目标窗口
        ret, track_window = cv2.meanShift(dst, self.ROI, self.term_crit)
        x, y, w, h = track_window
        # 更新ROI
        self.ROI[0] += (x - self.ROI[0]) * self.LOW_PASS_RATIO
        self.ROI[1] += (y - self.ROI[1]) * self.LOW_PASS_RATIO
        # 计算中心点坐标
        cx = int(x + w / 2)
        cy = int(y + h / 2)
        if _DEBUG:
            output_img = cv2.rectangle(frame, (x, y), (x + w, y + h), 255, 2)
            debug_imshow(output_img)
        return cx - self.img_shape[1] / 2, cy - self.img_shape[0] / 2

    def reset_roi(self, ROI: Optional[Tuple[Union[int, float], ...]] = None) -> None:
        """
        重置ROI
        """
        if ROI is None:
            ROI = self.init_ROI
        x, y, w, h = ROI
        if (x + y + w + h) <= 4:
            x = int(self.img_shape[1] * x)
            y = int(self.img_shape[0] * y)
            w = int(self.img_shape[1] * w)
            h = int(self.img_shape[0] * h)
        self.ROI = np.array([x, y, w, h])
        self.inited = False


__bs = None


def mixed_background_sub(frame) -> Tuple[bool, List[Tuple[float, float]]]:
    """
    混合高斯运动检测
    frame: 输入帧
    return: 是否检测到运动物体, 物体中点坐标列表
    """
    global __bs
    if __bs is None:
        __bs = cv2.createBackgroundSubtractorKNN(detectShadows=True)
    fgmask = __bs.apply(frame)
    th = cv2.threshold(fgmask, 244, 255, cv2.THRESH_BINARY)[1]  # 将非纯白色像素设为0
    th = cv2.erode(th, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)  # 腐蚀图像
    dilated = cv2.dilate(th, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (8, 3)), iterations=2)  # 膨胀处理
    contours, hier = cv2.findContours(
        dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detection_list = []
    for c in contours:
        if cv2.contourArea(c) > 1000:
            (x, y, w, h) = cv2.boundingRect(c)
            detection_list.append((x + w / 2, y + h / 2))
            if _DEBUG:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
    if _DEBUG:
        # debug_imshow( frame & cv2.cvtColor(fgmask, cv2.COLOR_GRAY2BGR))
        debug_imshow(th)
        debug_imshow(frame, 1)
    if len(detection_list) > 0:
        return True, detection_list
    return False, []


__hsv_map = None


def init_hsv_viewer():
    """
    (调试工具) 初始化HSV颜色直方图窗口
    """
    global __hsv_map
    __hsv_map = np.zeros((180, 256, 3), np.uint8)
    h, s = np.indices(__hsv_map.shape[:2])
    __hsv_map[:, :, 0] = h
    __hsv_map[:, :, 1] = s
    __hsv_map[:, :, 2] = 255
    __hsv_map = cv2.cvtColor(__hsv_map, cv2.COLOR_HSV2BGR)
    cv2.namedWindow("hsv_map", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("hsv_map", __hsv_map)
    cv2.namedWindow("hsv_hist", cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar("scale", "hsv_hist", 10, 32, lambda x: 0)


def update_hsv_viewer(img):
    """
    (调试工具) 更新HSV颜色直方图
    """
    global __hsv_map
    # small = cv2.pyrDown(img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 去除孤立的点
    dark = hsv[:, :, 2] < 32
    hsv[dark] = 0
    h = cv2.calcHist([hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
    hist_scale = cv2.getTrackbarPos("scale", "hsv_hist")
    h = np.clip(h * 0.005 * hist_scale, 0, 1)

    # 从一维变成三维
    # 将得到的直方图和颜色直方图相乘
    vis = __hsv_map * h[:, :, np.newaxis] / 255.0
    cv2.imshow("hsv_hist", vis)


def init_hsv_selector():
    """
    (调试工具) 初始化HSV颜色选择器
    """
    cv2.namedWindow("Selector", cv2.WINDOW_NORMAL)
    cv2.namedWindow("HSV_img", cv2.WINDOW_AUTOSIZE)
    def nothing(x): return 0
    cv2.createTrackbar("H_l", "Selector", 0, 255, nothing)
    cv2.createTrackbar("H_h", "Selector", 0, 255, nothing)
    cv2.createTrackbar("S_l", "Selector", 0, 255, nothing)
    cv2.createTrackbar("S_h", "Selector", 0, 255, nothing)
    cv2.createTrackbar("V_l", "Selector", 0, 255, nothing)
    cv2.createTrackbar("V_h", "Selector", 0, 255, nothing)


def update_hsv_selector(img):
    """
    (调试工具) 更新HSV颜色选择器
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (9, 9), 0)
    h_l = cv2.getTrackbarPos("H_l", "Selector")
    h_h = cv2.getTrackbarPos("H_h", "Selector")
    s_l = cv2.getTrackbarPos("S_l", "Selector")
    s_h = cv2.getTrackbarPos("S_h", "Selector")
    v_l = cv2.getTrackbarPos("V_l", "Selector")
    v_h = cv2.getTrackbarPos("V_h", "Selector")
    UPPER = np.array([h_h, s_h, v_h], dtype=np.uint8)
    LOWER = np.array([h_l, s_l, v_l], dtype=np.uint8)
    mask = cv2.inRange(hsv, LOWER, UPPER)
    cv2.imshow("HSV_img", mask)
    print(f"UPPER: {UPPER} LOWER: {LOWER}")


def change_cam_resolution(cam, width: int, height: int, fps: int = 60):
    """
    改变摄像头分辨率
    return 切换后的 宽,高,fps
    """
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cam.set(cv2.CAP_PROP_FPS, fps)
    return (
        cam.get(cv2.CAP_PROP_FRAME_WIDTH),
        cam.get(cv2.CAP_PROP_FRAME_HEIGHT),
        cam.get(cv2.CAP_PROP_FPS),
    )


def open_camera(try_from: int = 0, try_to: int = 10):
    """
    打开摄像头
    return 摄像头对象, 摄像头编号
    """
    cam = cv2.VideoCapture()
    for i in range(try_from, try_to):
        cam.open(i)
        if cam.isOpened():
            return cam, i
    raise Exception("Camera not found")


def open_camera_plus(cam_id=None, retries=3, warmup_frames=10, try_from: int = 0, try_to: int = 10):
    """带重试和预热的摄像头初始化"""
    if cam_id is None:
        cam = cv2.VideoCapture()
        for i in range(try_from, try_to):
            for attempt in range(retries):
                cam.open(i)
                if not cam.isOpened():
                    logger.error(f"尝试 {attempt+1}/{retries} 打开摄像头失败")
                    continue
                # 强制设置基础参数
                cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 最小缓冲
                # 帧预热
                for _ in range(warmup_frames):
                    ret, _ = cam.read()
                    if ret:
                        logger.success(f"摄像头{i}初始化成功")
                        return cam
                    time.sleep(0.1)
                cam.release()
            logger.warning(f"无法初始化摄像头{i}，已尝试 {retries} 次")

        raise Exception("Camera not found")
    else:
        for attempt in range(retries):
            cam = cv2.VideoCapture(cam_id)
            if not cam.isOpened():
                logger.error(f"尝试 {attempt+1}/{retries} 打开摄像头失败")
                continue
            # 强制设置基础参数
            cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 最小缓冲
            # 帧预热
            success_warmup_frames = 0
            for _ in range(warmup_frames):
                ret, _ = cam.read()
                if ret:
                    success_warmup_frames += 1
                time.sleep(0.05)  # 稍微短一点的延迟
            if success_warmup_frames >= warmup_frames * 0.8:  # 比如，至少80%的预热帧成功
                logger.success(f"摄像头{cam_id}初始化成功")
                return cam
            else:
                logger.warning(
                    f"摄像头{cam_id}预热帧读取不足，实际成功 {success_warmup_frames}/{warmup_frames}")
                cam.release()  # 预热不成功则释放
                continue  # 继续尝试下一个摄像头ID
        raise RuntimeError(f"无法初始化摄像头{cam_id}，已尝试 {retries} 次")


def rotate_img(image, angle, fill_color=(0, 0, 0)):
    """
    任意角度旋转图片
    angle: 旋转角度，顺时针方向, 角度制
    fill_color: 填充颜色
    """
    (h, w) = image.shape[:2]
    (cX, cY) = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))
    M[0, 2] += (nW / 2) - cX
    M[1, 2] += (nH / 2) - cY
    return cv2.warpAffine(image, M, (nW, nH), borderValue=fill_color)


def rotate_img_90(image, angle):
    """
    旋转图片90度
    angle: 旋转角度，顺时针方向, 角度制
    """
    if angle == 90:
        return cv2.transpose(image)
    elif angle == 180:
        return cv2.flip(image, -1)
    elif angle == 270:
        return cv2.transpose(image)
    else:
        return image


def set_cam_autowb(cam, enable=True, manual_temp=5500, hue=0):
    """
    设置摄像头自动白平衡
    enable: 是否启用自动白平衡
    manual_temp: 手动模式下的色温
    """
    cam.set(cv2.CAP_PROP_AUTO_WB, int(enable))
    if not enable:
        cam.set(cv2.CAP_PROP_WB_TEMPERATURE, manual_temp)


def set_cam_autoexp(cam, enable=True, manual_exposure=0.25):
    """
    设置摄像头自动曝光
    enable: 是否启用自动曝光
    manual_exposure: 手动模式下的曝光时间
    """
    cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, int(enable))
    if not enable:
        cam.set(cv2.CAP_PROP_EXPOSURE, manual_exposure)


class fps_counter:
    def __init__(self, max_sample=60) -> None:
        self.t = time.perf_counter()
        self.max_sample = max_sample
        self.t_list: List[float] = []

    def update(self) -> None:
        now = time.perf_counter()
        self.t_list.append(now - self.t)
        self.t = now
        if len(self.t_list) > self.max_sample:
            self.t_list.pop(0)

    @property
    def fps(self) -> float:
        length = len(self.t_list)
        sum_t = sum(self.t_list)
        if length == 0:
            return 0.0
        else:
            return length / sum_t


def stack_images(imgArray, scale=0.5, lables=[]) -> np.ndarray:
    """
    将多张图像合并成一张图像
    imgArray: 图像阵列 (单行 [img1, img2, img3, ...] 或多行 [[img11, img12,...], [img21, img22, ...], ...])
    lables: 图像标签阵列, 形式应与imgArray一致
    scale: 图像缩放比例
    """
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    blank_img = np.zeros_like(imgArray[0][0])
    if rowsAvailable:
        for r in range(rows):
            if len(imgArray[r]) != cols:
                diff = cols - len(imgArray[r])
                for _ in range(diff):
                    imgArray[r].append(blank_img)
        for x in range(0, rows):
            for y in range(0, cols):
                imgArray[x][y] = cv2.resize(
                    imgArray[x][y], (0, 0), None, scale, scale)
                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv2.cvtColor(
                        imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
            hor_con[x] = np.concatenate(imgArray[x])
        ver = np.vstack(hor)
        ver_con = np.concatenate(hor)
    else:
        for x in range(0, rows):
            imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)  # type: ignore
        hor_con = np.concatenate(imgArray)
        ver = hor  # type: ignore
    if len(lables) != 0:
        eachImgWidth = int(ver.shape[1] / cols)
        eachImgHeight = int(ver.shape[0] / rows)
        for d in range(0, rows):
            for c in range(0, cols):
                text = str(lables[d][c])
                text_size = cv2.getTextSize(
                    text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                cv2.rectangle(
                    ver,
                    (c * eachImgWidth, eachImgHeight * d),
                    (
                        c * eachImgWidth + text_size[0] + 10,
                        eachImgHeight * d + text_size[1] + 10,
                    ),
                    (0, 0, 0),
                    cv2.FILLED,
                )
                cv2.putText(
                    ver,
                    str(lables[d][c]),
                    (eachImgWidth * c + 5, eachImgHeight * d + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                )
    return ver


_ap_detector = None


def detect_apriltag(
    frame,
    nthreads: int = 1,
    quad_sigma: float = 0.0,
    sharpening: float = 0.25,
    families: str = "tag36h11",
    tag_size: Optional[float] = None,
    camera_params: Optional[Tuple[float, float, float, float]] = None,
) -> List[apriltag.Detection]:
    """
    AprilTag检测
    frame: 输入帧
    nthreads: 线程数
    quad_sigma: 四边形边缘模糊
    sharpening: 图像锐化
    families: AprilTag族, 空格分隔
    tag_size: AprilTag尺寸, 单位m
    camera_params: 相机参数 (fx, fy, cx, cy)
    return: 检测结果列表 [see https://pupil-apriltags.readthedocs.io/en/stable/api.html#]
    """
    global _ap_detector
    if _ap_detector is None:
        _ap_detector = apriltag.Detector(
            families=families,
            quad_sigma=quad_sigma,
            nthreads=nthreads,
            decode_sharpening=sharpening,
        )
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if camera_params is not None and tag_size is not None:
        results = _ap_detector.detect(
            gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)
    else:
        results = _ap_detector.detect(gray)
    if _DEBUG:
        img = frame.copy()
        for r in results:
            b = (tuple(r.corners[0].astype(int))[0],
                 tuple(r.corners[0].astype(int))[1])
            c = (tuple(r.corners[1].astype(int))[0],
                 tuple(r.corners[1].astype(int))[1])
            d = (tuple(r.corners[2].astype(int))[0],
                 tuple(r.corners[2].astype(int))[1])
            a = (tuple(r.corners[3].astype(int))[0],
                 tuple(r.corners[3].astype(int))[1])
            cv2.polylines(img, [np.array([a, b, c, d])], True, (0, 255, 0), 2)
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
            f = r.tag_family.decode("utf-8")
            i = r.tag_id
            cv2.putText(
                img,
                f"{f}:{i}",
                (a[0], a[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2,
            )
        debug_imshow(img)
    return results


def set_manual_exporsure(camera, exposure):
    camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    exposure = 2**exposure*10000
    camera.set(cv2.CAP_PROP_EXPOSURE, exposure)


class HighPrecisionFPS:

    def __init__(self):
        self._start_time = perf_counter()

    def reset(self):  # 添加一个重置计时起点的方法
        self._start_time = perf_counter()

    def fps(self) -> float:
        # 确保时间流逝，避免除以零
        elapsed_time = perf_counter() - self._start_time
        if elapsed_time == 0:
            return float('inf')  # 或者根据需要返回其他值
        return 1.0 / elapsed_time


def is_circle(contour, circularity_thresh=0.7):
    """
    根据圆形度阈值判断一个轮廓是否足够接近圆形。
    圆形度 C = 4 * pi * 面积 / (周长^2)，完美圆形 C 接近 1。

    Args:
        contour (np.ndarray): 输入轮廓。
        circularity_thresh (float, optional): 最小圆形度阈值。默认为 0.7。

    Returns:
        bool: 如果轮廓的圆形度高于阈值，则返回 True；否则返回 False。
    """
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)

    if perimeter == 0:
        return False

    circularity = 4 * np.pi * area / (perimeter ** 2)
    return circularity > circularity_thresh


def detect_blue_circles(frame, min_radius=10, max_radius=200, circularity_thresh=0.7,
                        gaussian_blur_ksize=(5, 5), morphology_kernel_size=(8, 8)):
    """
    在给定视频帧中检测蓝色圆环
    Args:
        frame (np.ndarray): 输入的 BGR 格式视频帧。
        min_radius (int, optional): 检测到的圆的最小半径。默认为 10 像素。
        max_radius (int, optional): 检测到的圆的最大半径。默认为 200 像素。
        circularity_thresh (float, optional): 轮廓的最小圆形度阈值。默认为 0.7。
        gaussian_blur_ksize (tuple, optional): 高斯模糊核的大小 (宽度, 高度)。宽度和高度必须是奇数。默认为 (5, 5)。
        morphology_kernel_size (tuple, optional): 形态学操作（闭运算和膨胀）核的大小 (宽度, 高度)。默认为 (8, 8)。

    Returns:
        list: 包含检测到的圆的元组列表，格式为 (x, y, radius)。
              其中 (x, y) 是圆心的整数坐标，radius 是其整数半径。
    """
    # 转换到 HSV 空间并提取蓝色区域
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 定义蓝色 HSV 范围，这些值可能需要根据实际情况调整
    lower_blue = np.array([95, 106, 40])
    upper_blue = np.array([111, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # 形态学处理：闭合断裂边缘
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, morphology_kernel_size)
    closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    # 应用高斯模糊以减少噪声
    blurred = cv2.GaussianBlur(closed, gaussian_blur_ksize, 0)
    # 边缘检测 + 膨胀连接
    edges = cv2.Canny(blurred, 50, 150)
    edges = cv2.dilate(edges, kernel, iterations=1)  # 膨胀也使用相同的形态学核
    # 轮廓检测 + 最小外接圆拟合
    contours, _ = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    circles = []
    for cnt in contours:
        # 确保轮廓有足够的点且符合圆形度
        if len(cnt) >= 5 and is_circle(cnt, circularity_thresh):
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            # 根据半径范围过滤
            if min_radius <= radius <= max_radius:
                circles.append((int(x), int(y), int(radius)))

    return circles


def find_largest_contour_info(image: np.ndarray, thr: int, di: bool = False, er: bool = False) -> Tuple[bool, Optional[int], Optional[int], Optional[float]]:
    """
    识别图像中的最大黑色轮廓并返回其中心坐标。
    可视化部分通过 _DEBUG 参数控制。

    Args:
        image (np.ndarray): 输入的图像帧 (BGR 格式)。
        _DEBUG (bool, optional): 是否显示调试用的可视化窗口和绘图。默认为 False。

    Returns:
        tuple: 
            - Optional[Tuple[int, int]]: 最大轮廓的中心坐标 (center_x, center_y)。如果未找到或计算失败，则为 None。
            - Optional[np.ndarray]: 绘制了轮廓和中心点的调试图像副本。如果 _DEBUG=False 或处理失败，则为 None。
    """
    if image is None:
        print("错误：输入图像为 None")
        return False, 0, 0, 0
    debug_image = image.copy() if _DEBUG else None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(
        blurred, thr, 255, cv2.THRESH_BINARY_INV)

    if di:
        thresh = cv2.dilate(thresh, (8, 8))  # 膨胀
    if er:
        thresh = cv2.erode(thresh, (5, 5))  # 腐蚀
    # 查找轮廓
    if _DEBUG:
        debug_imshow(thresh, "Process1")
    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center_coords: Optional[Tuple[int, int]] = None
    # 找到最大轮廓
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        contour_area = cv2.contourArea(largest_contour)
        # 计算中心点
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            center_coords = (center_x, center_y)
            if _DEBUG and debug_image is not None:
                # 绘制最大轮廓 (红色)
                cv2.drawContours(
                    debug_image, [largest_contour], -1, (0, 0, 255), 3)
                # 在中心点绘制一个圆 (黄色)
                cv2.circle(debug_image, center_coords, 5, (0, 255, 255), -1)
                # 在图像上显示中心点坐标
                cv2.putText(debug_image, f"Center: {center_coords}", (center_x + 10, center_y + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                debug_imshow(debug_image, "Process")
            return True, center_x-image.shape[1]//2, center_y-image.shape[0]//2, contour_area
        elif _DEBUG and debug_image is not None:
            cv2.putText(debug_image, "Largest contour area is zero", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return None, 0, 0, 0
    elif _DEBUG and debug_image is not None:
        cv2.putText(debug_image, "No contours found", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    debug_imshow(debug_image, "Process")

    return False, 0, 0, 0


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
    基于OpenCV轮廓检测,从图像中找到白色连通区域的中心
    Args:
        image: 输入图像 (可以是BGR三通道或灰度图)
        min_area: 最小轮廓面积
        max_area: 最大轮廓面积,如果为None,则不限制最大面积                  
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
        根据output_form返回:
        - 'both':   [{'y_kx_b': (k,b), 'x_ky_b': (k,b)}, ...]
        - 'y_kx_b': [(k,b), ...] 
        - 'x_ky_b': [(k,b), ...]
    """
    if lines is None:
        return []

    # 确保 lines 是一个可迭代的数组，即使只有一个线段
    if isinstance(lines, np.ndarray) and lines.ndim == 1 and lines.shape[0] == 4:
        lines_to_process = [lines]
    elif isinstance(lines, np.ndarray) and lines.ndim == 3 and lines.shape[2] == 2:
        lines_to_process = lines.reshape(-1, 2)
    elif isinstance(lines, np.ndarray) and lines.ndim == 3 and lines.shape[2] == 4:
        lines_to_process = lines.reshape(-1, 4)
    elif isinstance(lines, np.ndarray) and lines.ndim == 2 and lines.shape[1] == 4:
        lines_to_process = lines
    else:
        lines_to_process = lines

    equations = []

    for line_data in lines_to_process:
        rho, theta = 0.0, 0.0

        # 判断输入线段的格式 (rho, theta) 还是 (x1, y1, x2, y2)
        if len(line_data) == 2:
            rho, theta = line_data
        elif len(line_data) == 4:
            x1, y1, x2, y2 = line_data
            norm = np.sqrt((y2-y1)**2 + (x2-x1)**2)
            if norm < 1e-6:
                continue
            A = (y2-y1) / norm
            B = (x1-x2) / norm  # 这里是 x1-x2 而非 -(x2-x1) 因为我们需要 A*x + B*y = rho 的形式
            theta = np.arctan2(B, A)
            if theta < 0:
                theta += np.pi
            mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
            rho = mid_x * np.cos(theta) + mid_y * np.sin(theta)
            # 如果 rho 是负数，调整 theta 使 rho 为正 (OpenCV HoughLines 约定)
            if rho < 0:
                rho = -rho
                theta += np.pi
                if theta > np.pi:
                    theta -= np.pi
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
            y_kx_b = (np.inf, rho / a)

        # 处理x=ky+b形式
        if abs(a) > 1e-6:  # 非水平线 (斜率不为inf)
            k_x = -b / a
            b_x = rho / a
            x_ky_b = (k_x, b_x)
        else:  # 水平线 (y=const)
            x_ky_b = (np.inf, rho / b)

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


def get_area_in_frame(
        img: np.ndarray, bias: int = 20, skip_di: bool = False, skip_er: bool = False, kernal_size_di: int = 9, kernal_size_er: int = 5, hough_threshold: int = 80,
        min_line_length: int = 60, max_lne_gap: int = 200) -> List[Tuple[Optional[float], Optional[float], Optional[float]]]:
    """找到离中心最近的矩形，并去除外面的图像

    Args:
        bias: 往内缩小的像素数量
        skip_di: 是否跳过膨胀
        skip_er: 是否跳过腐蚀

    Returns:
        障碍物的位姿列表 (x,y,yaw) / pixels/deg。
        x_out: 障碍物到后侧线的距离。
        y_out: 障碍物到右侧线的距离。
        这里的yaw通常为None,因为单点障碍物无法确定朝向。
    """
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.Canny(img, 100, 170)
    kernel_di = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernal_size_di, kernal_size_di))
    kernel_er = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernal_size_er, kernal_size_er))

    processed_img = img.copy()
    if not skip_di:
        processed_img = cv2.dilate(processed_img, kernel_di)  # 膨胀
    if not skip_er:
        processed_img = cv2.erode(processed_img, kernel_er)  # 腐蚀
    lines_p = cv2.HoughLinesP(
        processed_img,
        1,
        np.pi / 180,
        threshold=hough_threshold,
        minLineLength=min_line_length,
        maxLineGap=max_lne_gap,
    )
    if _DEBUG:
        display_img = cv2.cvtColor(processed_img, cv2.COLOR_GRAY2BGR)
    else:
        display_img = None
    if lines_p is None:
        if _DEBUG:
            if display_img is not None:
                debug_imshow(display_img, "Result")
        return None
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

    if right_lines.shape[0] != 0:
        dists, _ = get_point_line_distance_np(
            [x_img_center, y_img_center], right_lines)
        line_index = np.argmin(dists)
        id_right_line = right_lines[line_index]
    if back_lines.shape[0] != 0:
        dists, _ = get_point_line_distance_np(
            [x_img_center, y_img_center], back_lines)
        line_index = np.argmin(dists)
        id_back_line = back_lines[line_index]
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
                             b_back-bias)
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
                              x_coords + b_front+bias)
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
                              y_coords + b_right-bias)
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
                mask_left = (x_coords < t_left * y_coords + b_left+bias)
                result[mask_left] = 0
            else:
                pass

    # 可视化部分
    if _DEBUG:
        # 1. 原始输入图像
        debug_imshow(img, "Origin")
        # 2. 膨胀/腐蚀后的图像
        debug_imshow(processed_img, "Process1")
        # 3. 原始 HoughLinesP 检测到的所有线段
        temp_all_hough_lines_img = cv2.cvtColor(
            processed_img, cv2.COLOR_GRAY2BGR)
        if lines_p is not None:
            for line in lines_p:
                x1, y1, x2, y2 = line[0]
                cv2.line(temp_all_hough_lines_img, (x1, y1),
                         (x2, y2), (0, 165, 255), 1)  # 橙色细线
        debug_imshow(temp_all_hough_lines_img, "Process2")
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
        debug_imshow(temp_ref_lines_img, "Process3")
        # 5. 掩模后的图像
        debug_imshow(result, "Result")

    return result


if __name__ == "__main__":
    # 测试
    FLANN_match(cv2.imread("test.jpg"), cv2.imread("test.jpg"))
    pass
