import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger

# camera, id = open_camera()
# print(f"Camera ID: {id}")
# get = change_cam_resolution(camera, 640, 480, 60)
# print(f"Info: {get[0]}x{get[1]} @ {get[2]}fps")
vision_debug(saveimg=True)
# set_cam_autowb(camera, True)

img = cv2.imread("data.jpg")
img = rescale_aspect_ratio(img, 1280, 720)


################ PROCESS ##############################
def get_img() -> np.ndarray:
    # return camera.read()[1]
    return img.copy()


def process(img: np.ndarray) -> np.ndarray:
    HOUGH_THRESHOLD = 80
    MIN_LINE_LENGTH = 60
    # KERNAL_DI = 9  # 膨胀核大小
    # KERNAL_ER = 5  # 腐蚀核大小
    # HOUGH_THRESHOLD = 50
    # MIN_LINE_LENGTH = 60
    MAX_LINE_GAP = 1
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # # 闭运算
    kernel = np.ones((5, 5), np.uint8)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    # # 二值化
    _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # # 反色
    img = cv2.bitwise_not(img)
    # img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    lines = cv2.HoughLinesP(
        img,
        1,
        np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=MIN_LINE_LENGTH,
        maxLineGap=MAX_LINE_GAP,
    )
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if lines is None:
        return img
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), 255, 1)

    return img


#######################################################
last_error = ""
cv2.namedWindow("Origin", cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("Processed", cv2.WINDOW_AUTOSIZE)
while True:
    rimg = get_img()
    if rimg is None:
        continue
    cv2.imshow("Origin", rimg)
    ################# PROCESS ##########################
    try:
        pimg = process(rimg)
        cv2.imshow("Processed", pimg)
    except Exception as e:
        if last_error != str(e):
            logger.exception(e)
            last_error = str(e)
    ####################################################
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
cv2.destroyAllWindows()
