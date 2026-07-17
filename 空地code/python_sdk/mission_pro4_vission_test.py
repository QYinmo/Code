import cv2
import numpy as np
from loguru import logger
from vision.Vision_plus import *

cam_begin = False
model = "/home/n1/workplace/Drone_maindev/python_sdk/robocup_best.pt"
conf_thres = 0.9
cam = open_camera_plus(0)
started = True
not_see = True


def camera_task(cam):
    res_pre = -1
    while True:
        ret, img = cam.read()
        # img = img[50:200, 100:300]
        blue, _, _, area = find_anycolor_area(img, np.array(
            [124, 255, 255]), np.array([100, 106, 46]))
        if cam_begin and blue and area > 500:
            identify_status = True
        else:
            identify_status = False
        if identify_status:
            res = identify_area(img, model)
            if res == -1 and not_see:
                continue
            else:
                not_see = False
                logger.info("停下导航")
                if res == res_pre:
                    area_count += 1
                    area_timeout = 0
                else:
                    area_timeout += 1
                    res_pre = res
                if area_count > 1:
                    area_count = 0
                    area_timeout = 0
                    identify_status = False
                    if started:
                        area_res = int(res)
                        if area_res == 0:
                            logger.info(f"发现帐篷")
                            navi.navigation_follow_trajectory(
                                navi.traj_list_before_stop, wait=False)
                            time.sleep(2)
                        if area_res == 1:
                            logger.info(f"发现地堡")
                            if not challenge:
                                drop_pack()
                                logger.info(
                                    "[MISSION] Pack dropped, continue trajectory")
                            navi.navigation_follow_trajectory(
                                navi.traj_list_before_stop, wait=False)
                            time.sleep(2)
                        if area_res == 4:
                            logger.info(f"发现桥")
                            drop_pack()
                            logger.info(
                                "[MISSION] Pack dropped, continue trajectory")
                            navi.navigation_follow_trajectory(
                                navi.traj_list_before_stop, wait=False)
                            time.sleep(2)
                        if area_res == 2:
                            logger.info(f"发现坦克")
                            if challenge:
                                tank_mession()
                                logger.info(
                                    "[MISSION] Pack dropped, continue trajectory")
                            navi.navigation_follow_trajectory(
                                navi.traj_list_before_stop, wait=False)
                            time.sleep(2)
                        if area_res == 3:
                            logger.info(f"发现车")
                            drop_pack()
                            logger.info(
                                "[MISSION] Pack dropped, continue trajectory")
                            navi.navigation_follow_trajectory(
                                navi.traj_list_before_stop, wait=False)
                            time.sleep(2)
                elif area_timeout > 2:
                    area_timeout = 0
                    area_count = 0
                    identify_status = False
                    not_see = True
                    if started:
                        area_res = -1
                        logger.warning("看错了")
                        navi.navigation_follow_trajectory(
                            navi.traj_list_before_stop, wait=False)
                        time.sleep(1)
        time.sleep(0.1)


def find_point(img):
    ret, img = cam.read()
    if ret:
        logger.warning("没图像")
        return False, 0, 0
    res, center_x, center_y = identify_area(img, model)
    if res != -1:

        return True, center_x - img.shape[1] / 2, center_y - img.shape[0] / 2
    else:
        logger.warning("没找到")
        return False, 0, 0


def identify_area(img, model):
    try:
        results = model(source=img, show=False, conf=conf_thres, save=False,
                        verbose=False, stream=True, device='cpu')
        processed_results = []
        final_results = []  # 用于存储最终结果
        if results is not None:
            for result in results:
                if result.boxes is not None and len(result.boxes.cpu().numpy()) > 0:
                    boxes = result.boxes.cpu().numpy()

                    normalized_xy = boxes.xywhn[:, :2]
                    classes = boxes.cls
                    confs = boxes.conf
                    annotated_frame = result.plot()  # 自动绘制boxes/labels
                    debug_imshow(annotated_frame, "Result")
                    for i in range(len(classes)):
                        processed_results.append([
                            normalized_xy[i][0],       # x坐标
                            normalized_xy[i][1],       # y坐标
                            int(classes[i]),            # 类别
                            float(confs[i])            # 置信度
                        ])

                    processed_results = np.array(processed_results)

                    cls_results = processed_results
                    max_conf_idx = np.argmax(cls_results[:, 3])
                    best_result = cls_results[max_conf_idx]
                    final_results.append(best_result)
                    final_results = [list(row) for row in final_results]
                    for res in final_results:
                        print(
                            f"类别: {int(res[2])}, 坐标: ({res[0]:.5f}, {res[1]:.5f}), 置信度: {res[3]:.5f}")
                        return res[2], (res[0]-0.5)*img.shape[0], (res[1]-0.5)*img.shape[1]
                else:
                    print("没东西")
                    return -1, 0, 0
    except:
        logger.warning("死了")
        return -1, 0, 0
