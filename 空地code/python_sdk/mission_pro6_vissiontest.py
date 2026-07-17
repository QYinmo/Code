# 寻宝无人机
import cv2
import numpy as np
from loguru import logger
from vision.Vision_plus import *
cap = cv2.VideoCapture(0)  # 0表示默认摄像头
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
cap.set(cv2.CAP_PROP_EXPOSURE, -5)
vision_debug()


def id_function(img):
    img = get_ROI(img, (0.5, 0.5, 0.7, 0.7))
    debug_imshow(img, "Origin")
    logger.debug("正在识别")
    f, dx, dy, res = color_recognition(img, 0.4)
    if f:
        logger.info(f"res:{res}")
        return True, dx, dy, res
    return False, 0, 0, 0


while True:
    ret, frame = cap.read()
    if not ret:
        break
    f, dx, dy, res = id_function(frame)
    logger.debug(f"f:{f}dx:{dx}dy:{dy}res:{res}")
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
