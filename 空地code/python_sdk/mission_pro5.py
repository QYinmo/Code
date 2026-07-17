"""
货架识别
"""
import random
import struct
import threading
import time
from typing import List
from FlightController.Components.RosManager import RosManager
import cv2
import numpy as np
from config_manager import ConfigManager
from FlightController import FC_Client, FC_Controller, FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Components.UartScreen import UARTScreen
from FlightController.Solutions.Navigation import Navigation
from FlightController.Solutions.PathPlanner import PFBPP, TrajectoryGenerator
from vision.Vision_plus import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger
from usr_serial import Serial_station, Serial_gpio
from simple_pid import PID

cfg = ConfigManager(section="mission")

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([250, -350])
SPEED = 20
HEIGHT = 150
X_SIZE = np.array([50, 0, 0])
Y_SIZE = np.array([0, -50, 0])
Z_SIZE = np.array([0, 0, 40])
def P(x, y, z): return np.array([25, -25, HEIGHT+40]) + X_SIZE * x + Y_SIZE * y- Z_SIZE * z


MISSION1_POINTS = np.array([
    P(1, 0, 1), P(2, 0, 1), P(3, 0, 1), P(3, 0, 0), P(2, 0, 0), P(1, 0, 0),
    P(-1, 0, 0), P(-1, -2, 0),  # 非识别点6，7
    P(1, -2, 0), P(2, -2, 0), P(3, -2, 0), P(3, -2, 1), P(2, -2, 1), P(1, -2, 1),
    P(1, -4, 1), P(2, -4, 1), P(3, -4, 1), P(3, -4, 0), P(2, -4, 0), P(1, -4, 0),
    P(-1, -4, 0), P(-1, -6, 0),  # 非识别点20，21
    P(1, -6, 0), P(2, -6, 0), P(3, -6, 0), P(3, -6, 1), P(2, -6, 1), P(1, -6, 1),
])
# 数字为巡航顺序，id是商品id
point = {
    # 第一组 (基础值0)
    1: {"position": P(1, 0, 1), "id": 3},
    2: {"position": P(2, 0, 1), "id": 2},
    3: {"position": P(3, 0, 1), "id": 1},
    4: {"position": P(3, 0, 0), "id": 4},
    5: {"position": P(2, 0, 0), "id": 5},
    6: {"position": P(1, 0, 0), "id": 6},

    # 第二组 (基础值6)
    7: {"position": P(1, -2, 0), "id": 10},
    8: {"position": P(2, -2, 0), "id": 11},
    9: {"position": P(3, -2, 0), "id": 12},
    10: {"position": P(3, -2, 1), "id": 9},
    11: {"position": P(2, -2, 1), "id": 8},
    12: {"position": P(1, -2, 1), "id": 7},

    # 第三组 (基础值12)
    13: {"position": P(1, -4, 1), "id": 15},
    14: {"position": P(2, -4, 1), "id": 14},
    15: {"position": P(3, -4, 1), "id": 13},
    16: {"position": P(3, -4, 0), "id": 16},
    17: {"position": P(2, -4, 0), "id": 17},
    18: {"position": P(1, -4, 0), "id": 18},

    # 第四组 (基础值18)
    19: {"position": P(1, -6, 0), "id": 22},
    20: {"position": P(2, -6, 0), "id": 23},
    21: {"position": P(3, -6, 0), "id": 24},
    22: {"position": P(3, -6, 1), "id": 21},
    23: {"position": P(2, -6, 1), "id": 20},
    24: {"position": P(1, -6, 1), "id": 19}
}


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.rs: T265 = kwargs["rs"]
        self.navi: Navigation = kwargs["navi"]
        self.screen: UARTScreen = kwargs.get("screen", None)

        self.x_pid = PID(Kp=0.2, Ki=0.001, Kd=0.05, setpoint=0)
        self.y_pid = PID(Kp=0.2, Ki=0.001, Kd=0.05, setpoint=0)
        self.x_pid.output_limits = (-50, 50)  # 限制输出范围
        self.y_pid.output_limits = (-50, 50)
        self.x_pid.sample_time = 0.08
        self.y_pid.sample_time = 0.08

        self.cam_direction = 1  # 当摄像头在机头方向右侧时为1否则-1
        self.point_num = 0  # 巡航到哪个了1~24
        self.location = 0
        self.identify_status = False
        self.takeoff_flag = False
        self.use_serial = True
        self.mode1_scaning = False
        self.mode1_path = np.array([])
        self.mode1_pos = ()
        self.mode1_done = False
        self.mode = 0
        self.mode0_state = False
        self.use_pid = False

        # 激光笔1，2，舵机1为左边，0为右边
        self.gpio_com = [170, 0, 0, 0,  255]
        # 商品坐标1~24—>A1~D6，商品编号1~24,任务2扫描的编号,任务2完成否
        self.terminal_com = [170, 0, 0, 0, 0, 255]
        # 是否起飞，任务编号,商品坐标1~24—>A1~D6
        self.rxbuffer = [0, 0, 0]
        self.QR_wait_event = threading.Event()
        self.next_point_event = threading.Event()
        self.takeoff_event = threading.Event()
        if self.use_serial:
            self.serial_terminal = Serial_station(
                device="cp2102", baudrate=115200, rx_length=5)
            self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)
        if self.fc.last_command_done:
            pass

    def generate_path(self):
        path = []
        # 起点 P(-1,0,1)
        current = P(-1, 0, 1)
        target_point = point[self.mode1_pos]["position"]
        path.append(current)
        target_x, target_y, target_z = target_point[0], target_point[1], target_point[2]
        current = P(0, target_y, 1)
        path.append(current)
        current = P(target_x, target_y, 1)
        path.append(current)
        current = P(target_x, target_y, target_z)
        path.append(current)  # 第4个是目的地
        # 终点P(5,-6,1)
        current = P(target_x, target_y, 1)
        path.append(current)
        current = P(5, target_y, 1)
        path.append(current)
        current = P(5, -6, 1)
        path.append(current)
        return np.array(path, dtype=object)

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def laser(self, status):
        if self.cam_direction == 1:
            num = 1
        else:
            num = 2
        self.gpio_com[num] = status
        if status:
            logger.info("[MISSION] Laser ON")
        else:
            logger.info("[MISSION] Laser OFF")

    def servo(self):
        self.gpio_com[3] = not self.gpio_com[3]

    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()

    def start_mode_handle(self):
        threading.Thread(target=self.mode_handle, daemon=True).start()

    def camera_task(self):
        data_pre = -1
        count = 0
        none_count = 0
        while True:
            ret, img = self.cam.read()
            if not ret:
                logger.warning("没有图像")
            if self.identify_status:
                f, _, _ = find_QRcode_contour(img)
                if f:
                    logger.info("找到疑似二维码")
                    if self.use_pid:
                        self.target()
                    else:
                        img = get_ROI(img, (0.27, 0.27, 0.46, 0.46))
                        exist, data = self.scan(img)
                        if exist:
                            none_count = 0
                            if data == data_pre:
                                count += 1
                            else:
                                count = 0
                            data_pre = data
                            if count > 1:
                                logger.info(f"读取完成data:{data}")
                                self.terminal_com[1] = point[self.point_num]["id"]
                                self.terminal_com[2] = data
                                count = 0
                                self.laser(True)
                                time.sleep(0.5)
                                self.laser(False)
                                if self.point_num in {6, 14, 20, 28}:
                                    self.cam_direction = -self.cam_direction
                                    self.servo()
                                self.next_point_event.set()
                        else:
                            none_count += 1
                            if none_count > 4:
                                none_count = 0
                                logger.warning("此处无码")
                                if self.point_num in {6, 14, 20, 28}:
                                    self.cam_direction = -self.cam_direction
                                    self.servo()
                                self.next_point_event.set()
            if self.mode1_scaning:
                exist, data = self.scan(img)
                if exist:
                    if data == data_pre:
                        count += 1
                    else:
                        count = 0
                    data_pre = data
                    if count > 3:
                        logger.info(f"读取完成data:{data}")
                        self.terminal_com[3] = data
                        count = 0

    def scan(self, img):
        debug_imshow(img, "Origin")
        exist, _, _, data = find_QRcode_zbar(img)
        if exist:
            logger.info(f"data:{data}")
            return True, data
        else:
            logger.warning("扫不出来")
        return False, None

    def target(self):
        time_found = 0
        state = 0
        navi = self.navi
        fc = self.fc
        fc.set_rgb_led(255, 255, 0)
        self.x_pid.reset()
        self.y_pid.reset()

        while True:
            _, frame = self.cam.read()
            if frame is None:
                continue

            img = get_ROI(frame, (0.27, 0.27, 0.46, 0.46))
            point = navi.current_point
            f, dx, dy = find_QRcode_contour(img)

            if f:

                if state == 0:
                    x_control = self.x_pid(dx)*self.cam_direction
                    z_control = self.y_pid(dy)
                    logger.info(
                        f"[MISSION] PID control: x_control={x_control}, z_control={z_control}")
                    if abs(dx) < 15 and abs(dy) < 15:
                        logger.info(f"[MISSION] Reached fire at {point}")
                        navi.set_height(80)
                        state = 1
                    to_point = (point[0] - x_control, point[1])

                    navi.direct_set_waypoint(to_point)

                if state == 1 and abs(navi.current_height - navi.height_pid.setpoint) < 8:
                    logger.info(f"[MISSION] Reached height at {point}")
                    time_found = time.perf_counter()
                    fc.set_rgb_led(255, 0, 0)
                    state = 2

                if state == 2 and time.perf_counter() - time_found > 3:
                    logger.info(f"[MISSION] Released pack at {point}")
                    fc.set_rgb_led(0, 0, 0)
                    navi.set_height(self.cruise_height)
                    navi.wait_for_height()

                    return

            time.sleep(0.08)

    def mode_handle(self):
        while True:
            if self.rxbuffer[0] and not self.takeoff_flag:
                self.takeoff_event.set()
                self.takeoff_flag = True
                break
            if self.rxbuffer[1] == 1 and not self.mode1_done:
                self.mode = 1
                time.sleep(0.5)
                self.QR_wait_event.clear()
                logger.info("任务2二维码识别中")
                self.fc.set_indicator_led(0, 0, 255)  # 识别中
                self.mode1_scaning = True
                self.QR_wait_event.wait()
                self.QR_wait_event.clear()
                time.sleep(0.5)
                while self.rxbuffer[2]:
                    self.mode1_pos = self.rxbuffer[2]
                logger.info("任务2可以做了")
                self.mode1_path = self.generate_path()
                self.mode1_done = True
            time.sleep(0.07)

    def run(self):
        fc = self.fc
        radar = self.radar
        cam = self.cam
        navi = self.navi
        ############### 参数 #################
        self.navigation_speed = SPEED  # 导航速度
        self.cruise_height = HEIGHT  # 巡航高度
        self.vertical_speed = 20  # 垂直速度
        self.send_restart = 0
        self.restart = 0
        ################ 启动线程 ################
        self.inital_yaw = self.fc.state.yaw.value
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()  # 启动导航线程
        navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started")
        ################  校准 ################
        navi.set_basepoint(BASE_POINT)
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        vision_debug(saveimg=True)
        fc.set_action_log(False)
        change_cam_resolution(cam, 640, 480, 60)
        set_cam_autowb(cam, True)
 #       set_manual_exporsure(cam, -10)
        fc.set_indicator_led(0, 255, 0)
        time.sleep(0.05)
        self.serial_gpio.send_start(self.gpio_com)
        self.serial_terminal.send_start(self.terminal_com)
        self.serial_terminal.listen_start(self.rxbuffer)
        self.start_camera_task()
        self.start_mode_handle()
        time.sleep(0.1)

        ################ 初始化完成 ################
        self.takeoff_event.clear()
        logger.info("[MISSION] Serial received . Waiting for takeoff signal")
        fc.set_indicator_led(255, 255, 255)  # 等待起飞命令
        self.takeoff_event.wait()
        self.takeoff_event.clear()
        self.started = True
        for _ in range(3):
            time.sleep(0.25)
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(0.25)
            fc.set_indicator_led(0, 0, 0)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")

        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        if self.mode == 0:
            logger.info("任务1开始")
            for self.location in range(len(MISSION1_POINTS)):
                navi.navigation_to_waypoint(MISSION1_POINTS[self.location])
                self.navi.wait_for_waypoint()
                logger.info(
                    f"[MISSION] Go to target point {MISSION1_POINTS[self.location]}")
                if self.location not in {6, 7, 20, 21}:
                    self.next_point_event.clear()
                    self.point_num += 1
                    self.identify_status = True
                    self.next_point_event.wait()
                    self.next_point_event.clear()
                    fc.set_indicator_led(0, 0, 255)  # 蓝灯提示
                    time.sleep(0.5)
                    fc.set_indicator_led(0, 0, 0)
        else:
            logger.info("任务2开始")
            if (1 <= self.mode1_pos <= 6) or (13 <= self.mode1_pos <= 18):
                self.gpio_com[3] = 0
            else:
                self.gpio_com[3] = 1
            for self.location in range(len(self.mode1_path)):
                navi.navigation_to_waypoint(self.mode1_path[self.location])
                self.navi.wait_for_waypoint()
                logger.info(
                    f"[MISSION] Go to target point {self.mode1_path[self.location]}")
                if self.location == 3:  # 目的地
                    time.sleep(1)
                    self.laser(True)
                    time.sleep(0.5)
                    self.laser(False)
                    self.terminal_com[4] = True
        navi.pointing_landing(LANDING_POINT)


if __name__ == "__main__":
    # fc = FC_Controller()
    # fc.start_listen_serial(print_state=True)
    fc = FC_Client()
    fc.connect()
    fc.wait_for_connection()
    t265 = T265("ros")
    t265.start()
    radar = LD_Radar()
    radar.start("ros")
    cam = open_camera_plus()

    time.sleep(0.1)
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
    time.sleep(0.5)
    fc.set_indicator_led(0, 0, 0)
    time.sleep(0.5)
    fc.close()
