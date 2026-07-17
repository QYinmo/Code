import cv2
import numpy as np
RED_UPPER = np.array([10, 255, 255])
RED_LOWER = np.array([0, 43, 46])
RED_UPPER2 = np.array([180, 255, 255])
RED_LOWER2 = np.array([156, 43, 46])
RED_UPPERF = np.array([180, 255, 255])
RED_LOWERF = np.array([150, 120, 90])

DEBUG = False

def red_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    """ mask1 = cv2.inRange(hsv, RED_LOWER, RED_UPPER)
    mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    res = mask1 + mask2 """
    mask = cv2.inRange(hsv, RED_LOWERF, RED_UPPERF)
    res = mask
    return res


def process_close(img, ek=3, dk=3, ie=1, id=1, it=1):
    dilate_kernel = np.ones((dk, dk), np.uint8)
    erode_kernel = np.ones((ek, ek), np.uint8)
    for i in range(it):
        img = cv2.dilate(img, dilate_kernel, iterations=id)
        img = cv2.erode(img, erode_kernel, iterations=ie)
    return img


def process_open(img, ek=3, dk=3, ie=1, id=1, it=1):
    dilate_kernel = np.ones((dk, dk), np.uint8)
    erode_kernel = np.ones((ek, ek), np.uint8)
    for i in range(it):
        img = cv2.erode(img, erode_kernel, iterations=ie)
        img = cv2.dilate(img, dilate_kernel, iterations=id)
    return img


def processing(img):
    img = red_mask(img)
    img = process_close(img, 5, 7, 5, 4, 1)
    return img


def detect_max_center(img):
    contours, _ = cv2.findContours(
        img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    # 找到最大的轮廓
    max_contour = max(contours, key=cv2.contourArea)
    # 计算最大的轮廓的中心
    M = cv2.moments(max_contour)
    if M["m00"] == 0:
        return None
    center_x = int(M["m10"] / M["m00"])
    center_y = int(M["m01"] / M["m00"])
    if DEBUG:
        debug_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(debug_img, [max_contour], -1, (0, 0, 255), 2)
        cv2.circle(debug_img, (center_x, center_y), 5, (0, 255, 0), -1)
        cv2.imshow('debug_img', debug_img)
    return (center_x, center_y)


def get_fire_loc(img):
    img = processing(img)
    res = detect_max_center(img)
    return res


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480.0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    while cap.isOpened():
        ret, img = cap.read()
        res = get_fire_loc(img)
        cv2.imshow('img', img)
        print(res)
        cv2.waitKey(20)
