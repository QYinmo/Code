"""
魔改中
"""
import random
import threading
import time
from typing import List

import cv2
import numpy as np
from FlightController import FC_Client, FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Components.UartScreen import UARTScreen
from FlightController.Solutions.Navigation import Navigation
# from FlightController.Solutions.PathPlanner import TrajectoryGenerator
from FlightController.Solutions.Vision_Net import FastestDetOnnx
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger
from trash.pid import pid
from trash.usr_serial import Serial_car, Serial_gpio
from onnx_detect import infer_and_draw

import onnxruntime as ort

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
SPEED = 30
HEIGHT = 120
def P(x, y): return np.array([x * 90+60, -y * 90-60, HEIGHT])


terrain = ['FARMLAND', 'LAKE', 'MUDSLIDE',
           'HILLFIRE', 'MOUNTAIN', 'VILLAGE', 'RIVER']

mission_points_list = []
for i in range(0, 3):
    mission_points_list.append(P(i, 0))
for i in range(2, -1, -1):
    mission_points_list.append(P(i, 1))
for i in range(0, 3):
    mission_points_list.append(P(i, 2))
for i in range(2, -1, -1):
    mission_points_list.append(P(i, 3))

MISSION_POINTS = np.array(mission_points_list)


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.rs: T265 = kwargs["rs"]
        self.navi: Navigation = kwargs["navi"]
        self.screen: UARTScreen = kwargs.get("screen", None)
        self.takeoff_event = threading.Event()
        self.use_serial = True
        self.started = False
        self.terminal_com = [170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255]
        self.gpio_com = [170, 1, 0, 255]
        self.terminal_rxbuffer = [0]
        self.session = ort.InferenceSession("./model/detect.onnx")
        self.last_point = None
        self.mud_point = None
        self.fire_point = None
        self.lake_point = None

        if self.use_serial:
            self.serial_terminal = Serial_car(
                device="cp2102", baudrate=115200, rx_length=3)
            self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)
        if self.fc.last_command_done:
            pass

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def detect(self, img):
        img_center_x = int(img.shape[1]/2)
        img_center_y = int(img.shape[0]/2)
        boxes, labels = infer_and_draw(img, self.session, 0.5)
        for label, box in zip(labels, boxes):
            center_x = int((box[0] + box[2]) / 2)
            center_y = int((box[1] + box[3]) / 2)
            if int((abs(center_x - img_center_x)**2+abs(center_y - img_center_y)**2)**(1/2)) < 100:
                # logger.info(f"label:{label}")
                print(center_x, center_y)
                return label

        return None

    def report(self, i, res):
        self.terminal_com[1] += 1
        self.terminal_com[i+1] = res
        # return None

    def rx_scan_task(self):
        while True:
            if self.terminal_rxbuffer[0] is not 0:
                self.takeoff_event.set()
                break
            # print(self.terminal_rxbuffer)
            time.sleep(0.1)

    def start_rx_scan_task(self):
        threading.Thread(target=self.rx_scan_task, daemon=True).start()

    def mud_handler(self):
        self.navi.navigation_stop_here()
        self.navi.set_height(80)
        self.navi.wait_for_height()
        # 投放物资 懒得做
        time.sleep(2)
        self.navi.set_height(HEIGHT)
        self.navi.wait_for_height()

    def fire_handler(self):
        # 闪烁并鸣笛报警
        self.navi.pointing_landing(self.lake_point)
        self.navi.pointing_takeoff(self.lake_point, HEIGHT)
        self.navi.direct_set_waypoint(self.fire_point)
        self.navi.wait_for_waypoint(time_thres=1, pos_thres=7, timeout=10)
        # 再次鸣笛
        self.navi.set_height(100)
        self.navi.wait_for_height()
        time.sleep(2)
        self.navi.set_height(HEIGHT)
        self.navi.wait_for_height()

    def run(self):
        fc = self.fc
        # radar = self.radar
        cam = self.cam
        navi = self.navi
        ############### 参数 #################
        self.camera_down_pwm = 65
        self.camera_up_pwm = 28
        self.navigation_speed = SPEED  # 导航速度
        self.cruise_height = HEIGHT  # 巡航高度
        self.vertical_speed = 20  # 垂直速度
        ################ 启动线程 ################
        self.inital_yaw = self.fc.state.yaw.value
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()  # 启动导航线程
        navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started")
        self.serial_terminal.start_transmit(
            self.terminal_com, self.terminal_rxbuffer)
        self.serial_gpio.send_start(self.gpio_com)
        self.start_rx_scan_task()
        ################  校准 ################
        navi.set_basepoint(BASE_POINT)
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(False)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480.0)
        # cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        # cam.set(cv2.CAP_PROP_EXPOSURE, 650)
        time.sleep(0.25)
        self.takeoff_event.clear()
        logger.info("[MISSION] Waiting for takeoff signal")
        self.takeoff_event.wait()
        self.takeoff_event.clear()
        self.started = True
        logger.info("[MISSION] Mission Started")
        for _ in range(2):
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(1)
            fc.set_indicator_led(0, 0, 0)  # 起飞前警告
            time.sleep(1)
        fc.set_action_log(True)
        ################ 初始化完成 ################
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        count = 0
        for point in mission_points_list:
            point_res = 160
            logger.info(f"move to {point}")
            navi.direct_set_waypoint(point)
            navi.wait_for_waypoint(time_thres=1, pos_thres=7, timeout=10)
            time.sleep(0.5)
            label_count = [0, 0, 0, 0, 0, 0, 0]
            for i in range(15):
                time.sleep(0.02)
                logger.info(f"count {i}")
                try:
                    res, img = cam.read()
                except:
                    logger.error("[MISSION] Camera read failed")
                if res:
                    det_res = self.detect(img)
                    if det_res is not None:
                        if det_res != self.last_point:
                            label_count[det_res] += 2
                        else:
                            label_count[det_res] += 1
                    else:
                        logger.error("[MISSION] No RES")
                else:
                    logger.error("[MISSION] No IMG")
            print(
                f"max:{max(label_count)},label_count:{label_count.index(max(label_count))}")
            non_zero_count = sum(1 for x in label_count if x != 0)
            if non_zero_count == 1 and max(label_count) > 3:
                point_res = label_count.index(max(label_count)) + 1
            elif non_zero_count > 1 and max(label_count) > 6:
                point_res = label_count.index(max(label_count)) + 1
            count += 1
            self.report(count, point_res)
            self.last_point = point_res
            if point_res == 1:
                self.lake_point = point
                if self.fire_point is not None:
                    self.fire_handler()  # 永远只会在后出现的点触发
            if point_res == 2:
                self.mud_point = point
                self.mud_handler()
            if point_res == 3:
                self.fire_point = point
                if self.lake_point is not None:
                    self.fire_handler()  # 不用额外写判断
        navi.pointing_landing(LANDING_POINT)


if __name__ == "__main__":
    logger.warning("DEBUG MODE!!")
    # fc = FC_Controller()
    # fc.start_listen_serial(print_state=True)
    fc = FC_Client()
    fc.connect()
    fc.wait_for_connection()
    t265 = T265("ros")
    t265.start()
    radar = LD_Radar()
    radar.start("ros")
    cam, i = open_camera()
    logger.info(f"Camera {i} opened")
    screen = UARTScreen(fc)
    mapper = RosMapper()
    navi = Navigation(
        fc=fc,
        rs=t265,
        radar=radar,
        mapper=mapper,
    )
    RosNodeRunner().add_nodes().run()

    mission = Mission(
        fc=fc,
        cam=cam,
        rs=t265,
        radar=radar,
        navi=navi,
        mapper=mapper,
        screen=screen,
    )
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
