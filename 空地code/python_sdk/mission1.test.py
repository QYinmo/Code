"""
使用雷达作为位置闭环的任务模板
"""
import threading
import time
from typing import List

import cv2
import numpy as np
from configManager import ConfigManager
from FlightController import FC_Client, FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Solutions.Navigation import Navigation
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger


def deg_360_180(deg):
    if deg > 180:
        deg = deg - 360
    return deg


cfg = ConfigManager(section="mission")

BASE_CALI_POINT = cfg.get_array("point-base", default=np.array([81.28021476, 418.3601995]))

BASE_POINT = np.array([0, 0])
LANDING_POINT = np.array([0, 0])
# 任务坐标
NULL_PT = np.array([np.NaN, np.NaN])
POINT = lambda x: cfg.get_array(f"point-{x}")
POINTS_ARR = np.array(
    [
        [POINT(1), NULL_PT, POINT(11), NULL_PT, POINT(5)],
        [NULL_PT, POINT(8), NULL_PT, POINT(3), NULL_PT],
        [POINT(9), NULL_PT, POINT(2), NULL_PT, POINT(12)],
        [NULL_PT, POINT(7), NULL_PT, POINT(6), NULL_PT],
        [NULL_PT, NULL_PT, POINT(10), NULL_PT, POINT(4)],
    ]
)
logger.info(f"[MISSION] Loaded points: {POINTS_ARR}")
# 任务点
RED_TRIANGLES = [(0, 0), (2, 2)]
RED_RECTANGLES = [(0, 2), (2, 4)]
RED_CIRCLES = [(4, 0), (3, 3)]
BLUE_TRIANGLES = [(3, 1), (4, 4)]
BLUE_RECTANGLES = [(2, 0), (4, 2)]
BLUE_CIRCLES = [(1, 1), (1, 3)]
target_points = [cfg.get_array("target-1"), cfg.get_array("target-2")]
logger.info(f"[MISSION] Loaded target points: {target_points}")


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture, rs: T265):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        self.rs = rs
        self.inital_yaw = self.fc.state.yaw.value
        self.navi = Navigation(fc, radar, rs)
        self.fd = FastestDetOnnx(drawOutput=True)  # 初始化神经网络

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def run(self):
        fc = self.fc
        radar = self.radar
        cam = self.cam
        navi = self.navi
        ############### 参数 #################
        self.camera_down_pwm = 67
        self.camera_up_pwm = 28
        self.navigation_speed = 50  # 导航速度
        self.precision_speed = 20  # 精确速度
        self.cruise_height = 140  # 巡航高度
        self.goods_height = 80  # 处理物品高度
        self.vertical_speed = 20  # 垂直速度
        ################ 启动线程 ################
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()  # 启动导航线程
        navi.switch_navigation_mode("fusion")
        logger.info("[MISSION] Navigation started")
        ################  校准 ################
        navi.set_basepoint(BASE_CALI_POINT)
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(False)
        change_cam_resolution(cam, 640, 480, 60)
        set_cam_autowb(cam, True)
        fc.set_PWM_output(0, self.camera_up_pwm)
        self.recognize_targets()
        fc.set_PWM_output(0, self.camera_down_pwm)
        fc.set_indicator_led(0, 255, 0)
        fc.event.key_short.clear()
        fc.event.key_short.wait_clear()
        fc.set_digital_output(0, False)  # 激光笔
        for _ in range(6):
            time.sleep(0.25)
            fc.set_digital_output(1, True)  # 蜂鸣器
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(0.25)
            fc.set_digital_output(1, False)  # 蜂鸣器
            fc.set_indicator_led(0, 0, 0)
        fc.set_action_log(True)
        # navi.calibrate_realsense(False)
        ################ 初始化完成 ################
        logger.info("[MISSION] Mission-1 Started")
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        ################ 开始任务 ################
        for target_point in target_points:
            x, y = target_point
            target_point_pos = POINTS_ARR[y, x]
            navi.navigation_to_waypoint(target_point_pos)
            navi.wait_for_waypoint()
            self.handle_goods()
        ################ 降落 ################
        logger.info("[MISSION] Go to landing point")
        navi.navigation_to_waypoint(LANDING_POINT)
        navi.wait_for_waypoint()
        navi.pointing_landing(LANDING_POINT)
        logger.info("[MISSION] Misson-1 Finished")

    def handle_goods(self):
        """
        处理物品
        """
        logger.info(f"[MISSION] Handle goods")
        self.navi.set_height(self.goods_height)
        self.navi.wait_for_height()
        #####################################
        self.fc.set_pod(1, 8500)
        time.sleep(8)
        ####################################
        self.fc.set_indicator_led(0, 255, 0)
        self.fc.set_digital_output(0, True)
        self.fc.set_digital_output(1, True)
        time.sleep(5)
        self.fc.set_digital_output(0, False)
        self.fc.set_digital_output(1, False)
        self.fc.set_indicator_led(0, 0, 0)
        #####################################
        self.fc.set_pod(2, 10000)
        time.sleep(8)
        ####################################
        self.navi.set_height(self.cruise_height)
        self.navi.wait_for_height()

    def recognize_targets(self):
        global target_points
        original_target_points = target_points
        target_points = []
        self.fc.event.key_double.clear()
        logger.info("[MISSION] Recognizing targets")
        self.fc.set_indicator_led(255, 255, 0)
        rec_dict = {}
        start = False
        last_scan_time = time.time()
        while True:
            img = self.cam.read()[1]
            if img is None:
                continue
            get = self.fd.detect(img)
            for res in get:
                name = res[1]
                logger.debug(f"[MISSION] Recognized target: {name}")
                rec_dict[name] = rec_dict.get(name, 0) + 1
                if not start:
                    start = True
                last_scan_time = time.time()
                self.fc.set_indicator_led(
                    int(name[0] == "r") * 255, int(name[0] == "g") * 255, int(name[0] == "b") * 255
                )
            if start and time.time() - last_scan_time > 2:
                break
            if self.fc.event.key_double.is_set():  # 按键双击
                target_points = original_target_points
                return
        max_idx = max(rec_dict, key=rec_dict.get)
        logger.info(f"[MISSION] Final target: {max_idx}")
        if max_idx == "r_rec":
            target_points = RED_RECTANGLES[:]
        elif max_idx == "b_rec":
            target_points = BLUE_RECTANGLES[:]
        elif max_idx == "r_tri":
            target_points = RED_TRIANGLES[:]
        elif max_idx == "b_tri":
            target_points = BLUE_TRIANGLES[:]
        elif max_idx == "r_cir":
            target_points = RED_CIRCLES[:]
        elif max_idx == "b_cir":
            target_points = BLUE_CIRCLES[:]
        logger.info("[MISSION] Set target points: {}".format(target_points))


if __name__ == "__main__":
    logger.warning("DEBUG MODE!!")
    # fc = FC_Controller()
    # fc.start_listen_serial(print_state=True)
    # fc.wait_for_connection()
    fc = FC_Client()
    fc.connect()
    t265 = T265()
    t265.start()
    t265.hardware_reset()
    radar = LD_Radar()
    radar.start(fc)
    cam, i = open_camera()
    logger.info(f"Camera {i} opened")

    mission = Mission(fc, radar, cam, t265)  # type: ignore
    # logger.warning("Press Enter to start mission")
    # input()
    try:
        mission.run()
    except Exception as e:
        logger.exception(f"[MANAGER] Mission Failed")
    finally:
        mission.stop()
        if fc.state.unlock.value:
            logger.warning("[MANAGER] Auto Landing")
            fc.set_flight_mode(fc.PROGRAM_MODE)
            fc.stablize()
            fc.land()
            ret = fc.wait_for_lock()
            if not ret:
                fc.lock()
    logger.info("[MANAGER] Mission finished")
    fc.set_indicator_led(0, 255, 0)
    fc.set_digital_output(1, True)
    time.sleep(0.5)
    fc.set_digital_output(1, False)
    fc.set_indicator_led(0, 0, 0)
    time.sleep(1)
    fc.close()
