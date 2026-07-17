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
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger
from trash.usr_serial import Serial_car, Serial_gpio
# from pyzbar.pyzbar import decode
BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([250, -350])
# 任务坐标
SPEED = 60
HEIGHT = 120
SERVO_CAM_LEFT = 0
SERVO_CAM_RIGHT = 180

Y_A = -25
Y_B = -125
Y_C = -225
Y_D = -325
mission_points_list = []
def P(x, y, z): return np.array([x * 50+25, -y * 175, z*35+90])


def point_arrange(y, z):
    for i in range(1, 4):
        mission_points_list.append(P(i, y, z))
    for i in range(3, 0, -1):
        mission_points_list.append(P(i, y, 1-z))


def change_dimension(y, z):
    mission_points_list.append(P(-1, y, 1-z))
    mission_points_list.append(P(-1, y+1, 1-z))


point_arrange(0, 0)
change_dimension(0, 0)
point_arrange(1, 1)
point_arrange(1, 0)
change_dimension(1, 0)
point_arrange(2, 1)


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
        self.QR_wait_event = threading.Event()
        self.next_point_event = threading.Event()
        self.mode = 0
        self.use_serial = True
        self.started = False
        self.terminal_com = [170,
                             0,  # 已扫描的货物
                             0, 0, 0, 0, 0, 0,  # A
                             0, 0, 0, 0, 0, 0,  # B
                             0, 0, 0, 0, 0, 0,  # C
                             0, 0, 0, 0, 0, 0,  # D
                             0,  # 任务2需要前往的点
                             255]
        self.gpio_com = [170, 0, 180, 0, 0, 255]  # 左激光 舵机 右激光
        self.terminal_rxbuffer = [0, 0, 0]
        self.identify_status = False
        self.QR_count = 0
        self.QR_timeout = 0
        self.QR_res = 160
        self.mode1_point = None
        self.mode1_done = False
        # if self.use_serial:
        #     self.serial_terminal = Serial_car(
        #         device="cp2102", baudrate=115200, rx_length=5)
        #     self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)
        # if self.fc.last_command_done:
        #     pass

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")


    def run(self):
        fc = self.fc
        # radar = self.radar
        cam = self.cam
        navi = self.navi
        ############### 参数 #################
        self.navigation_speed = SPEED  # 导航速度
        self.cruise_height = HEIGHT  # 巡航高度
        self.vertical_speed = 30  # 垂直速度
        ################ 启动线程 ################
        self.inital_yaw = self.fc.state.yaw.value
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()  # 启动导航线程
        navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started")
        time.sleep(2)
        # self.serial_terminal.start_transmit(
        #     self.terminal_com, self.terminal_rxbuffer)
        # self.serial_gpio.send_start(self.gpio_com)
        # self.start_rx_scan_task()
        ################  校准 ################
        navi.set_basepoint(BASE_POINT)
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(True)
        # cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
        # cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480.0)
        # cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        # cam.set(cv2.CAP_PROP_EXPOSURE, 650)
        # self.start_camera_task()
        # while (self.serial_terminal.is_listened is False):
        #     continue

        # self.takeoff_event.clear()
        # logger.info("[MISSION] Serial received . Waiting for takeoff signal")
        # self.takeoff_event.wait()
        # self.takeoff_event.clear()
        self.started = True
        logger.info("[MISSION] Mission Started")
        for _ in range(3):
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(1)
            fc.set_indicator_led(0, 0, 0)  # 起飞前警告
            time.sleep(1)
        fc.set_action_log(True)
        ################ 初始化完成 ################
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        count = 0
        right_pose = [0, -150]
        base = [0, 0]
        for i in range(10):
            navi.direct_set_waypoint(right_pose)
            navi.wait_for_waypoint(timeout=15)
            navi.direct_set_waypoint(base)
            navi.wait_for_waypoint(timeout=15)
            logger(navi.navi_y_pid.Kp, navi.navi_y_pid.Ki, navi.navi_y_pid.Kd)
            
            # navi.set_height(70)
            # navi.wait_for_height(timeout=10)
            # time.sleep(5)
            # navi.set_height(150)
            # navi.wait_for_height(timeout=10)
            # time.sleep(5)



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
