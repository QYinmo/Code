import cv2
import numpy as np
from .pid import pid
from loguru import logger
""" RED_UPPER = np.array([10, 255, 255])
RED_LOWER = np.array([0, 43, 46])
RED_UPPER2 = np.array([180, 255, 255])
RED_LOWER2 = np.array([156, 43, 46]) """
RED_UPPERF = np.array([180, 255, 240])
RED_LOWERF = np.array([150, 150, 120])#搜索光源时需要拉大VL
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
    img = process_close(img, 5, 7, 3, 4, 1)
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
        #print(max_contour)
        debug_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(debug_img, [max_contour], -1, (0, 0, 255), 2)
        cv2.circle(debug_img, (center_x, center_y), 5, (0, 255, 0), -1)
        cv2.imshow('debug_img', debug_img)
    return (center_x, center_y)

count=0
def find_fire(img):
    global count
    try:
        img = processing(img)
        res = detect_max_center(img)
        img_center = (img.shape[1] // 2, img.shape[0] // 2)
        if res is not None:
            count += 1
        else:
            count = 0
        if count > 3:
            return img_center ##作为PID设定目标
        else:
            return None
    except:
        logger.warning("no img")

def get_fire_loc(img):#返回值为右x下y 匿名坐标系为上x左y 列表第二个直接赋给x速度 列表第一个的负值直接赋给y速度
    try:
        img = processing(img)
        res = detect_max_center(img)
        if res is not None:
            return res
        else:
            return None
    except:
        logger.warning("no img")
if __name__ == '__main__':
    DEBUG = True
    flag=0
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480.0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,1)
    cap.set(cv2.CAP_PROP_EXPOSURE, 32)
    while cap.isOpened():
        ret, img = cap.read()
        if flag==0:
            res=find_fire(img)
            if res != None:
                pid=pid([res[0],res[1]])
                flag=1
                print("central"+str(res))
        else:
            res=get_fire_loc(img)
            if res == None:
                continue
            else:
                spped=pid.get_cv_pid(res)#xy速度
                print(spped)
        cv2.imshow('img', img)
        cv2.waitKey(20)
