from typing import Optional

import cv2
import numpy as np

cv2.namedWindow("test", cv2.WINDOW_AUTOSIZE | cv2.WINDOW_KEEPRATIO)
import traceback


def draw(img: np.ndarray) -> np.ndarray:
    img = cv2.imread("test.jpg")
    blk = np.zeros_like(img)
    blk = cv2.rectangle(blk, (0, 0), (400, 400), (255, 255, 255), -1)
    img = cv2.addWeighted(img, 1, blk, -0.2, 0)
    return img


last_excption = ""
while True:
    try:
        img = np.zeros((720, 960, 3), np.uint8)
        img = draw(img)
        cv2.imshow("test", img)
    except Exception as e:
        if last_excption != str(e):
            traceback.print_exc()
            last_excption = str(e)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cv2.destroyAllWindows()
