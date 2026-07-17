import cv2
from vision.Vision_plus import vision_debug, open_camera_plus, debug_imshow, find_QRcode_contour, find_QRcode_zbar
from loguru import logger

vision_debug()
cam = open_camera_plus()
while True:
    ret, img = cam.read()
    if not ret:
        logger.warning("摄像头无图像")
        break
    debug_imshow(img, "Origin")
    f, _, _ = find_QRcode_contour(img)
    if f is True:
        exist, _, _, data = find_QRcode_zbar(img)
        if exist:
            logger.info(f"data:{data}")
        else:
            logger.info("None")
    if cv2.waitKey(1) == ord('q'):
        break
