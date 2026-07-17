from Vision_plus import debug_imshow, vision_debug
import cv2
import numpy as np
import pygame
import sys
from loguru import logger


def adjust_red_threshold(auto_expo):
    # 初始化 PyGame
    pygame.init()
    pygame.display.set_mode((1, 1))  # 创建最小化窗口用于键盘事件处理

    # 初始化摄像头
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 450)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 250)
    if not auto_expo:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # 关闭自动曝光

    # 创建 OpenCV 窗口
    cv2.namedWindow("HSV Threshold", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("HSV Threshold", 600, 400)

    # 创建滑动条
    cv2.createTrackbar("H1_min", "HSV Threshold", 125, 179, lambda x: None)
    cv2.createTrackbar("H1_max", "HSV Threshold", 155, 179, lambda x: None)
    cv2.createTrackbar("H2_min", "HSV Threshold", 160, 179, lambda x: None)
    cv2.createTrackbar("H2_max", "HSV Threshold", 179, 179, lambda x: None)
    cv2.createTrackbar("S_min", "HSV Threshold", 43, 255, lambda x: None)
    cv2.createTrackbar("V_min", "HSV Threshold", 46, 255, lambda x: None)
    cv2.createTrackbar("S_max", "HSV Threshold", 255, 255, lambda x: None)
    cv2.createTrackbar("V_max", "HSV Threshold", 255, 255, lambda x: None)
    cv2.createTrackbar("Exposure", "HSV Threshold", 0, 255, lambda x: None)

    saved_params = None  # 初始化保存的参数

    while True:
        ret, frame = cap.read()
        if not ret:
            logger.warning("摄像头没开")
            break

        # 获取滑动条值
        exposure = cv2.getTrackbarPos("Exposure", "HSV Threshold")
        exposure = np.interp(exposure, [0, 255], [-10, 10])  # 映射到 [-10, 10]
        h1_min = cv2.getTrackbarPos("H1_min", "HSV Threshold")
        h1_max = cv2.getTrackbarPos("H1_max", "HSV Threshold")
        h2_min = cv2.getTrackbarPos("H2_min", "HSV Threshold")
        h2_max = cv2.getTrackbarPos("H2_max", "HSV Threshold")
        s_min = cv2.getTrackbarPos("S_min", "HSV Threshold")
        v_min = cv2.getTrackbarPos("V_min", "HSV Threshold")
        s_max = cv2.getTrackbarPos("S_max", "HSV Threshold")
        v_max = cv2.getTrackbarPos("V_max", "HSV Threshold")

        # 设置摄像头曝光
        if not auto_expo:
            cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        exposure0 = cap.get(cv2.CAP_PROP_EXPOSURE)
        print(exposure0)

        # 生成 HSV 掩模
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([h1_min, s_min, v_min])
        upper_red1 = np.array([h1_max, s_max, v_max])
        lower_red2 = np.array([h2_min, s_min, v_min])
        upper_red2 = np.array([h2_max, s_max, v_max])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

        # 轮廓检测
        contours, _ = cv2.findContours(
            img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        MIN_AREA = 150
        filtered_contours = [
            cnt for cnt in contours if cv2.contourArea(cnt) >= MIN_AREA]
        result_image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(result_image, filtered_contours, -1, (0, 255, 0), 2)

        # 显示图像
        debug_imshow(frame, "Origin")
        debug_imshow(mask, "Process")
        debug_imshow(result_image, "Result")

        # 处理 PyGame 事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:  # ESC 退出
                    cap.release()
                    cv2.destroyAllWindows()
                    pygame.quit()
                    return saved_params if saved_params is not None else (h1_min, h1_max, h2_min, h2_max, s_min, s_max, v_min, v_max, exposure)
                elif event.key == pygame.K_SPACE:  # 空格保存
                    saved_params = (h1_min, h1_max, h2_min, h2_max,
                                    s_min, s_max, v_min, v_max, exposure)
                    print("参数已保存:", saved_params)

        # OpenCV 窗口刷新
        cv2.waitKey(10)

    cap.release()
    cv2.destroyAllWindows()
    pygame.quit()
    return saved_params if saved_params is not None else (h1_min, h1_max, h2_min, h2_max, s_min, s_max, v_min, v_max, exposure)


if __name__ == "__main__":
    vision_debug()
    adjust_red_threshold(False)
