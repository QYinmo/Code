import cv2
import numpy as np
from loguru import logger
from Vision_plus import vision_debug, get_area_in_frame, get_ROI, debug_imshow, find_red_area
from ultralytics import YOLO
cap = cv2.VideoCapture(1)  # 0表示默认摄像头
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_EXPOSURE, -7)
vision_debug()

model = YOLO(
    r"/home/n1/workplace/Drone_maindev/python_sdk/?")  # 到时候请改为飞机电脑路径

ANIMAL_CLSASS = {
    0: "象",
    1: "虎",
    2: "狼",
    3: "猴",
    4: "孔雀",
}


def id_function(img, model):
    try:
        debug_imshow(img, "Origin")
        results = model(source=img, show=False, conf=self.conf_thres, save=False,
                        verbose=False, stream=True, device='cpu')
        all_detected_objects = []
        if results is not None:
            for result in results:
                if result.boxes is not None and len(result.boxes.cpu().numpy()) > 0:
                    boxes = result.boxes.cpu().numpy()
                    xy = boxes.xywh[:, :2]
                    classes = boxes.cls
                    confs = boxes.conf
                    annotated_frame = result.plot()  # 自动绘制boxes/labels
                    debug_imshow(annotated_frame, "Result")
                    for i in range(len(classes)):
                        all_detected_objects.append([
                            xy[i][0]-img.shape[1],       # all_detected_objects
                            xy[i][1]-img.shape[0],       # class_counts
                            int(classes[i]),            # 类别
                            float(confs[i])            # 置信度
                        ])
                else:
                    pass
        if not all_detected_objects:
            logger.info("没有检测到任何目标")
            return False, [], {}  # 返回False，空列表，空字典
        class_counts = {}
        for obj in all_detected_objects:
            class_id = obj[2]  # 类别ID
            class_counts[class_id] = class_counts.get(class_id, 0) + 1
        for obj in all_detected_objects:
            logger.info(
                f"类别: {ANIMAL_CLSASS[int(obj[2])]}, 坐标: ({obj[0]:.5f}, {obj[1]:.5f}), 置信度: {obj[3]:.5f}")

        logger.info("\n--- 类别统计 ---")
        for cls_id, count in class_counts.items():
            logger.info(f"类别 {ANIMAL_CLSASS[cls_id]}: {count} 个")
        return True, all_detected_objects, class_counts
    except Exception as e:
        logger.error(f"识别失败: {e}")
        logger.warning("死了")
        return False, [], {}


while True:
    ret, frame = cap.read()
    if not ret:
        break
    # res = get_area_in_frame(frame, -20, False, False, 5, 5, 110, 150, 20)
    f, all_detected_objects, class_counts = id_function(frame, model)
    logger.debug(
        f"f:{f}all_detected_objects:{all_detected_objects}class_counts:{class_counts}area:{area}")
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
