from Vision_plus import debug_imshow, vision_debug, set_manual_exporsure
import cv2
import numpy as np
from loguru import logger
from typing import List, Optional, Tuple, Union
cap = cv2.VideoCapture(1)
# set_manual_exporsure(cap, -10)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 450)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 250)

_DEBUG = True


def find_red_area(img) -> Tuple[bool, float, float, float]:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    RED_UPPER = np.array([8, 255, 255])
    RED_LOWER = np.array([0, 100, 80])
    RED_UPPER2 = np.array([180, 255, 255])
    RED_LOWER2 = np.array([160, 100, 80])
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
    contours, _ = cv2.findContours(
        img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    MIN_AERA = 120
    # filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= MIN_AERA]
    areas = [cv2.contourArea(cnt) for cnt in contours]
    sorted_contours = sorted(zip(areas, contours),
                             key=lambda x: x[0], reverse=True)
    filtered_contours = [cnt for area,
                         cnt in sorted_contours if area >= MIN_AERA]
    filtered_areas = [area for area,
                      cnt in sorted_contours if area >= MIN_AERA]
    if _DEBUG:
        result_image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(result_image, filtered_contours, -1, (0, 255, 0), 2)
        debug_imshow(result_image, "Result")
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
                debug_imshow(result_image, idx=2)
            return True, center_x - img.shape[1] / 2, center_y - img.shape[0] / 2, area
    return False, 0, 0, 0


vision_debug()
while True:
    ret, img = cap.read()
    if not ret:
        logger.warning("摄像头没开")
    debug_imshow(img, "Origin")
    f, dx, dy, s = find_red_area(img)
    logger.info(f"找到火源{f}位置{dx, dy}面积{s}")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
