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
from FlightController.Solutions.PathPlanner import TrajectoryGenerator
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger
from trash.vision_fire import get_fire_loc, find_fire
from trash.pid import pid
from trash.usr_serial import Serial_car, Serial_gpio

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
SPEED = 25
HEIGHT = 180
def P(x, y): return np.array([x * 72+5, -y * 72-10, HEIGHT])


mission_points_list = []
mission_points_list.append(P(0, 5))
mission_points_list.append(P(1, 5))
mission_points_list.append(P(1, 0))
mission_points_list.append(P(2, 0))
mission_points_list.append(P(2, 5))
mission_points_list.append(P(3, 5))
mission_points_list.append(P(3, 0))
mission_points_list.append(P(4, 0))
mission_points_list.append(P(4, 5))

MISSION_POINTS = np.array(mission_points_list)
DT = 0.1
MISSION_TRAJ: List[Tuple[float, float, float]] = []
MISSION_TRAJ.append((0, 0, HEIGHT))
TRAJ_LENGTH = 0.0
for i in range(len(MISSION_POINTS) - 1):
    last_p = MISSION_POINTS[i]
    next_p = MISSION_POINTS[(i + 1)]
    length = np.linalg.norm(next_p - last_p)
    TRAJ_LENGTH += float(length)
    traj_g = TrajectoryGenerator(last_p, next_p, length / SPEED)
    traj_g.solve()
    t = 0.0
    while t < length / SPEED:
        MISSION_TRAJ.append(traj_g.calc_position_xyz(t))
        t += DT
MISSION_TRAJ.append((0, 0, HEIGHT))
TRAJ_LENGTH += random.randint(-100, 100) / 10


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.rs: T265 = kwargs["rs"]
        self.navi: Navigation = kwargs["navi"]
        self.screen: UARTScreen = kwargs.get("screen", None)
        self.takeoff_event = threading.Event()
        self.use_serial = False
        self.started = False
        self.terminal_com = [170, 0, 0, 0, 0, 255]
        self.gpio_com = [170, 0, 0, 0, 0, 0, 1, 0, 0, 255]
        self.terminal_rxbuffer = [0, 0]
        if self.use_serial:
            self.serial_terminal = Serial_car(
                port="/dev/ttyUSB1", baudrate=115200, rx_length=4)
            self.serial_gpio = Serial_gpio(port="/dev/ttyUSB2", baudrate=38400)
        if self.fc.last_command_done:
            pass

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

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
        self.serial_terminal.start_transmit(self.terminal_com,self.terminal_rxbuffer)
        self.serial_gpio.send_start(self.gpio_com)
        ################  校准 ################
        navi.set_basepoint(BASE_POINT)
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(False)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480.0)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cam.set(cv2.CAP_PROP_EXPOSURE, 35)
        time.sleep(0.25)
        self.takeoff_event.clear()
        self.takeoff_event.wait()
        self.takeoff_event.clear()
        self.started = True
        for _ in range(2):
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(1)
            fc.set_indicator_led(0, 0, 0)  # 起飞前警告
            time.sleep(1)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")
        ################ 初始化完成 ################
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        navi.navigation_follow_trajectory(
            self.navi.traj_list_before_stop, wait=True)
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
