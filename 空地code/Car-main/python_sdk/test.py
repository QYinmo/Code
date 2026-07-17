import cv2
import numpy as np
from typing import Optional, Tuple


def find_largest_contour_info(image: np.ndarray, debug: bool = False) -> Tuple[Optional[Tuple[int, int]], Optional[np.ndarray]]:
    """
    识别图像中的最大轮廓并返回其中心坐标。
    可视化部分通过 debug 参数控制。

    Args:
        image (np.ndarray): 输入的图像帧 (BGR 格式)。
        debug (bool, optional): 是否显示调试用的可视化窗口和绘图。默认为 False。

    Returns:
        tuple: 
            - Optional[Tuple[int, int]]: 最大轮廓的中心坐标 (center_x, center_y)。如果未找到或计算失败，则为 None。
            - Optional[np.ndarray]: 绘制了轮廓和中心点的调试图像副本。如果 debug=False 或处理失败，则为 None。
    """
    if image is None:
        print("错误：输入图像为 None。")
        return None, None

    # 创建一个可供调试用的图像副本，避免修改原始图像
    debug_image = image.copy() if debug else None

    # 1. 预处理
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 2. 查找轮廓
    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    center_coords: Optional[Tuple[int, int]] = None

    # 3. 找到最大轮廓
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)

        # 4. 计算中心点
        M = cv2.moments(largest_contour)

        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            center_coords = (center_x, center_y)

            # 5. 可视化 (仅在 debug 模式下执行)
            if debug and debug_image is not None:
                # 绘制最大轮廓 (红色)
                cv2.drawContours(
                    debug_image, [largest_contour], -1, (0, 0, 255), 3)
                # 在中心点绘制一个圆 (黄色)
                cv2.circle(debug_image, center_coords, 5, (0, 255, 255), -1)
                # 在图像上显示中心点坐标
                cv2.putText(debug_image, f"Center: {center_coords}", (center_x + 10, center_y + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        elif debug and debug_image is not None:
            cv2.putText(debug_image, "Largest contour area is zero", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    elif debug and debug_image is not None:
        cv2.putText(debug_image, "No contours found", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    return center_coords, debug_image

# --- 摄像头测试功能 ---


def main_camera_test(debug: bool = False):
    """
    打开摄像头并实时识别最大轮廓的中心。
    """
    cap = cv2.VideoCapture(0)  # 尝试打开默认摄像头

    if not cap.isOpened():
        print("错误：无法打开摄像头。请检查摄像头是否连接正确或是否被其他程序占用。")
        return

    print("摄像头已打开。按 'q' 键退出。")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("错误：无法读取帧，可能是摄像头断开或视频流结束。")
            break

        # 调用识别函数，并根据 debug 参数决定是否生成调试图像
        center, debug_frame = find_largest_contour_info(frame, debug=debug)

        # 显示主摄像头画面
        # 如果debug，显示debug_frame，否则显示原始帧副本
        display_frame = debug_frame if debug_frame is not None else frame.copy()

        # 在主显示帧上添加额外的文字信息（如果需要）
        if center:
            cv2.putText(display_frame, f"Detected Center: {center}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "No object detected", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("Live Camera Feed (Press 'q' to quit)", display_frame)

        # 按 'q' 键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()


# --- 主程序入口 ---
if __name__ == '__main__':
    # 你可以通过修改这里的 debug=True/False 来控制是否显示调试信息
    # debug_mode = True  # 开启调试模式，会显示额外的窗口和绘图
    debug_mode = True  # 关闭调试模式，只显示基本的摄像头画面

    main_camera_test(debug=debug_mode)
