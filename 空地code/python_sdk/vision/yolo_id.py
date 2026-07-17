import torch
from PIL import Image
import sys
import os
import numpy as np
from pathlib import Path
from typing import Union
import cv2
from ultralytics import YOLO
from Vision_plus import *
timer = HighPrecisionFPS()
# 训练好的模型权重路径
model = YOLO(
    r"E:\yolov8\ultralytics-8.2.0\ultralytics-8.2.0\runs\detect\train4\weights\best.pt")
# 测试图片的路径
cap = cv2.VideoCapture(0)  # 0 是摄像头的设备索引，如果你有多个摄像头，可能需要改变这个值
cap.set(cv2.CAP_PROP_EXPOSURE, -8)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 450)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 250)
set_manual_exporsure
while True:
    ret, im0 = cap.read()  # 从摄像头读取一帧
    if not ret:
        break
    # 获取预测结果
    results = model(im0, stream=False)
    boxes = results[0].boxes  # 获取框的信息
    print(boxes)
    ann = results[0].plot()
    # 输出预测的类别和位置
    # for i, pred in enumerate(boxes.xywh):  # 访问xywh属性
    #     x, y, w, h = pred.tolist()  # 获取框的位置
    #     conf = scores[i].item()  # 获取当前框的置信度
    #     cls = classes[i].item()  # 获取当前框的类别索引

    #     print(f"类别：{model.names[int(cls)]}")  # 输出类别名称
    #     print(f"位置：x={x}, y={y}, w={w}, h={h}")  # 输出框的位置和尺寸
    #     print(f"置信度：{conf}")  # 输出置信度

    cv2.imshow("yolo", ann)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    print(f"瞬时FPS: {timer.fps():.1f}")
    timer.reset()
# 设置保存图片的路径
# cur_path = sys.path[0]
# print(cur_path, sys.path)

# if os.path.exists(cur_path):
#     cv2.imwrite(cur_path + os.sep + "out.jpg", ann)
# else:
#     os.mkdir(cur_path)
#     cv2.imwrite(cur_path + os.sep + "out.jpg", ann)
