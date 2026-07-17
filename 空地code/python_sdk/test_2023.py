import cv2

# 打开摄像头，参数0通常代表默认摄像头
cap = cv2.VideoCapture(1)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# --- 修改摄像头参数 ---

# 设置分辨率为 1280x720

# 设置帧率（FPS）为 30
# 注意：摄像头硬件必须支持该帧率，否则可能不会生效
cap.set(cv2.CAP_PROP_FPS, 120)

# 设置亮度为 100 (0-255)
# 注意：此参数可能因摄像头型号而异，有些摄像头可能不支持
cap.set(cv2.CAP_PROP_BRIGHTNESS, 100)

# 设置对比度为 50 (0-255)
cap.set(cv2.CAP_PROP_CONTRAST, 50)

# --- 验证参数是否设置成功 ---
print("当前分辨率:", cap.get(cv2.CAP_PROP_FRAME_WIDTH), "x", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("当前帧率:", cap.get(cv2.CAP_PROP_FPS))
print("当前亮度:", cap.get(cv2.CAP_PROP_BRIGHTNESS))

# --- 显示视频流 ---
while True:
    # 读取一帧
    ret, frame = cap.read()

    # 如果无法读取，则退出循环
    if not ret:
        print("无法接收帧 (流结束?)。退出 ...")
        break

    # 显示结果帧
    cv2.imshow('Camera Feed', frame)

    # 按下 'q' 键退出
    if cv2.waitKey(1) == ord('q'):
        break

# 释放摄像头并关闭窗口
cap.release()
cv2.destroyAllWindows()
