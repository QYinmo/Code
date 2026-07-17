import time
import cv2
from vision.Vision_plus import find_red_area, HighPrecisionFPS, set_manual_exporsure
import numpy as np
from loguru import logger
from get_cam_index import get_usb_2_0_camera, get_sy_1080p_camera
import os
H_PRIME_FILE = 'H_prime_LS_matrix_new.npy'
H_PRIME_MATRIX = None  # 初始化为 None

try:
    if not os.path.exists(H_PRIME_FILE):
        # 如果文件不存在，抛出错误，但我们会捕获并礼貌地提示用户
        raise FileNotFoundError(f"错误：未找到透视矩阵文件 '{H_PRIME_FILE}'。请先运行第一个代码文件。")

    # 加载高精度二进制矩阵
    H_PRIME_MATRIX = np.load(H_PRIME_FILE)

except FileNotFoundError as e:
    # 打印错误信息
    logger.info(e)

timer = HighPrecisionFPS()
cap1 = cv2.VideoCapture(get_usb_2_0_camera())
cap2 = cv2.VideoCapture(get_sy_1080p_camera())
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 360.0)
set_manual_exporsure(cap1, -5.5)
cv2.namedWindow('Camera1 Feed (Resizable)', cv2.WINDOW_NORMAL)
cv2.namedWindow('Camera2 Feed (Resizable)', cv2.WINDOW_NORMAL)


def transform_dxdy_to_fxfy_homography(dx: float, dy: float) -> Tuple[float, float]:
    """
    使用 Homography 将图像坐标 (dx, dy) 转换为现实坐标 (f_x, f_y)。
    """

    # 检查矩阵是否加载成功
    if H_PRIME_MATRIX is None:
        logger.warning("矩阵加载失败")
        return 0, 0

    # 输入点必须是 (1, 1, 2) 的 float64 数组，这是 cv2.perspectiveTransform 的要求
    image_point = np.array([[[dx, dy]]], dtype=np.float64)

    # 执行透视变换: H' 将 (dx, dy) 转换为 (f_y, f_x)
    world_point_transformed = cv2.perspectiveTransform(
        image_point, H_PRIME_MATRIX)

    fy = world_point_transformed[0, 0, 0]*100.0
    fx = world_point_transformed[0, 0, 1]*100.0-27.0  # -27是摄像头和飞机的相对位置

    return -fy, fx


while True:

    ret1, frame1 = cap1.read()
    if not ret1:
        print("无法接收帧1")
    if ret1:
        f, dx, dy, area = find_red_area(frame1, 4)
        logger.info(f"dx{dx},dy{dy},area{area}")
        fx, fy = transform_dxdy_to_fxfy_homography(dx, dy)
        logger.info(f"fx:{fx},fy:{fy}")
        # 显示帧
        cv2.imshow('Camera1 Feed (Resizable)', frame1)
    logger.info(f"瞬时FPS: {timer.fps():.1f}")
    timer.reset()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap1.release()
while True:

    ret2, frame2 = cap2.read()
    if not ret2:
        print("无法接收帧2")
    if ret2:
        f, dx, dy, area = find_red_area(frame2, 4)
        logger.info(f"dx{dx},dy{dy},area{area}")
        # 显示帧
        cv2.imshow('Camera2 Feed (Resizable)', frame2)
    logger.info(f"瞬时FPS: {timer.fps():.1f}")
    timer.reset()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap2.release()
cv2.destroyAllWindows()
