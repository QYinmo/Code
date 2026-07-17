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
from pyzbar.pyzbar import decode
BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([245, -350])
# 任务坐标
SPEED = 35
HEIGHT = 150
SERVO_CAM_LEFT = 0
SERVO_CAM_RIGHT = 180

Y_A = -25
Y_B = -125
Y_C = -225
Y_D = -325

mission_points_list = []

mission_2_points_list = []

def P(x, y, z): 
    return np.array([x * 50+25, -y * 175, z*40+85])


def point_arrange(y, z):
    for i in range(1, 4):
        mission_points_list.append(P(i, y, z))
    for i in range(3, 0, -1):
        mission_points_list.append(P(i, y, 1-z))


def change_dimension(y, z):
    mission_points_list.append(P(-1, y, 1-z))
    mission_points_list.append(P(-1, y+1, 1-z))


def mission2_arrange(id):
    mission_2_points_list.append(P(-1, 0, 1))  # 我直接后退
    print(f"ID={id}")
    if 1 <= id <= 6:
        mission_2_points_list.append(P(-1, 0, 1))  # 第一路 A
        mission_2_points_list.append(mission_points_list[id-1])  # 直接过去不就完了
        mission_2_points_list.append(P(5, 0, 0))  # 都飞一样高算了
        logger.info("第一路 A")
    elif 7 <= id <= 18:  # 第二路 BC共用
        mission_2_points_list.append(P(-1, 1, 1))
        mission_2_points_list.append(mission_points_list[id+1])
        mission_2_points_list.append(P(5, 1, 0))
        logger.info("第二路 BC")
    elif 19 <= id <= 24:  # 第三路 D
        mission_2_points_list.append(P(-1, 2, 1))
        mission_2_points_list.append(mission_points_list[id+3])
        logger.info("第三路 D")
    mission_2_points_list.append(P(5, 2, 0))


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
                             160,  # 任务2需要前往的点
                             255]
        self.gpio_com = [170, 0, 90, 0, 0, 255]  # 左激光 舵机 右激光
        self.terminal_rxbuffer = [0, 0, 0]
        self.identify_status = False
        self.QR_count = 0
        self.QR_timeout = 0
        self.QR_res = 160
        self.mode1_scaning = False
        self.mode1_res = None
        self.mode1_point = None
        self.mode1_done = False
        if self.use_serial:
            self.serial_terminal = Serial_car(
                device="cp2102", baudrate=115200, rx_length=5)
            self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)
        if self.fc.last_command_done:
            pass

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def identify_QR(self, img):
        try:
            image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            decoded_objects = decode(image)

            height, width = image.shape[:2]
            center_x = width // 2
            center_y = height // 2

            min_distance = float('inf')
            nearest_qr = None

            for obj in decoded_objects:
                qr_center_x = obj.rect.left + obj.rect.width // 2
                qr_center_y = obj.rect.top + obj.rect.height // 2

                distance = ((qr_center_x - center_x) ** 2 +
                            (qr_center_y - center_y) ** 2) ** 0.5

                if distance < min_distance:
                    min_distance = distance
                    nearest_qr = obj
            if nearest_qr is not None:
                return nearest_qr.data.decode('utf-8')
            else:
                return None
        except:
            logger.warning("死了")
            return None

    def report(self, i, res):
        self.terminal_com[1] += 1
        self.terminal_com[i+1] = res
        # return None

    def rx_scan_task(self):
        while True:
            if self.terminal_rxbuffer[1] != 0 and self.mode1_done is False:
                self.mode = 1
                time.sleep(0.3)
                self.QR_wait_event.clear()
                logger.info("接收到模式切换命令，二维码识别中")
                self.fc.set_indicator_led(0, 0, 255)  # 识别中
                self.mode1_scaning = True
                self.QR_wait_event.wait()
                self.QR_wait_event.clear()
                self.mode1_res = self.QR_res
                self.terminal_com[-2] = self.mode1_res
                logger.info("二维码识别完成，已发送，等待返回坐标")
                self.fc.set_indicator_led(0, 255, 0)
                while (self.terminal_rxbuffer[2] == 160):
                    continue
                time.sleep(1)
                self.mode1_point = self.terminal_rxbuffer[2]
                logger.info(f"point={self.mode1_point}")
                mission2_arrange(self.mode1_point)
                print(mission_2_points_list)
                logger.info("已得到返回坐标，准备进行任务2")
                self.mode1_done = True
                self.fc.set_indicator_led(255, 255, 255)
            if self.terminal_rxbuffer[0] is not 0:
                self.takeoff_event.set()
                break
            # print(self.terminal_rxbuffer)
            time.sleep(0.07)

    def camera_task(self):
        while True:
            ret, img = self.cam.read()
            if self.identify_status:
                res = self.identify_QR(img)
                logger.info(f"QR:{res}")
                if res is not None:
                    self.QR_count += 1
                    self.QR_timeout = 0
                else:
                    self.QR_timeout += 1
                if self.QR_count > 3:
                    self.QR_count = 0
                    self.QR_timeout =0
                    self.identify_status = False
                    if self.started:
                        self.QR_res = int(res)
                        self.next_point_event.set()
                elif self.QR_timeout > 15:
                    self.QR_timeout = 0
                    self.QR_count=0
                    self.identify_status = False
                    if self.started:
                        self.QR_res = 160
                        self.next_point_event.set()
            if self.mode1_scaning:
                res = self.identify_QR(img)
                if res is not None:
                    self.QR_count += 1
                else:
                    self.QR_count = 0
                if self.QR_count > 5:
                    self.QR_count = 0
                    self.QR_res = int(res)
                    self.mode1_scaning = False
                    self.QR_wait_event.set()

            time.sleep(0.06)

    def start_rx_scan_task(self):
        threading.Thread(target=self.rx_scan_task, daemon=True).start()

    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()

    def identify_once(self):
        self.identify_status = True

    def gpio_set_servo(self, side):
        if side == 0:
            self.gpio_com[2] = SERVO_CAM_RIGHT
        else:
            self.gpio_com[2] = SERVO_CAM_LEFT

    def gpio_set_laser(self, side, status):
        if side == 0:
            self.gpio_com[3] = status
            self.gpio_com[1] = 0
        else:
            self.gpio_com[1] = status
            self.gpio_com[3] = 0
    def carto_check(self):
        while True:
            if self.mode ==0:
                if (self.navi.mapper._trans_node.transform_established is False):
                    self.fc.set_indicator_led(255, 255, 0)
                    logger.info("我操你妈")
                else:
                    self.fc.set_indicator_led(255, 255, 255)
                    logger.info("你妈活了")
            if self.started:
                break
            time.sleep(0.3)
    def run(self):
        fc = self.fc
        # radar = self.radar
        cam = self.cam
        navi = self.navi
        ############### 参数 #################
        self.navigation_speed = SPEED  # 导航速度
        self.cruise_height = HEIGHT  # 巡航高度
        self.vertical_speed = 35  # 垂直速度
        ################ 启动线程 ################
        self.inital_yaw = self.fc.state.yaw.value
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()  # 启动导航线程
        navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started")
        time.sleep(2)
        self.serial_terminal.start_transmit(
            self.terminal_com, self.terminal_rxbuffer)
        self.serial_gpio.send_start(self.gpio_com)
        self.start_rx_scan_task()
        threading.Thread(target=self.carto_check, daemon=True).start()
        ################  校准 ################
        navi.set_basepoint(BASE_POINT)
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(True)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480.0)
        # cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        # cam.set(cv2.CAP_PROP_EXPOSURE, 650)
        self.start_camera_task()
        while (self.serial_terminal.is_listened is False):
            fc.set_indicator_led(0,255,0)
            time.sleep(0.13)

        self.takeoff_event.clear()
        logger.info("[MISSION] Serial received . Waiting for takeoff signal")
        fc.set_indicator_led(255, 255, 255)  # 等待起飞命令
        self.takeoff_event.wait()
        self.takeoff_event.clear()
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
        self.gpio_set_servo(0)
        if self.mode == 0:  # 任务1
            count = 0
            self.gpio_set_servo(0)
            for point in mission_points_list:
                if not (np.array_equal(point, mission_points_list[6]) or
                        np.array_equal(point, mission_points_list[7]) or
                        np.array_equal(point, mission_points_list[20]) or
                        np.array_equal(point, mission_points_list[21])):
                    count += 1
                    side = ((count-1)//6) % 2
                    self.QR_res = 160
                    logger.info(f"move to {point}")
                    navi.direct_set_waypoint(point)
                    if count == 4 or count == 16:
                        navi.wait_for_height(time_thres=1, timeout=15)
                    else:
                        navi.wait_for_height(time_thres=0.05, timeout=15)
                    navi.wait_for_waypoint(
                        time_thres=1.5, pos_thres=7, timeout=15)
                    self.next_point_event.clear()
                    self.identify_once()
                    self.next_point_event.wait()
                    self.next_point_event.clear()
                    res = self.QR_res
                    self.report(count, res)
                    self.gpio_set_laser(side, 1)
                    time.sleep(0.8)
                    self.gpio_set_laser(side, 0)
                    if count == 12:
                        self.gpio_set_servo(0)
                        time.sleep(0.8)
                else:
                    time.sleep(0.5)
                    if np.array_equal(point, mission_points_list[6]) or np.array_equal(point, mission_points_list[20]):
                        self.gpio_set_servo(1)
                    logger.info(f"move to {point}")
                    navi.direct_set_waypoint(point)
                    navi.wait_for_height(time_thres=0.05, timeout=10)
                    navi.wait_for_waypoint(
                        time_thres=1.5, pos_thres=7, timeout=10)
        else:
            logger.info("这就是任务2")
            id = self.mode1_point
            count = 0
            self.QR_res = 160
            if (1 <= id <= 6) or (13 <= id <= 18):
                side = 0
                self.gpio_set_servo(0)
            else:
                side = 1
                self.gpio_set_servo(1)
            if len(mission_2_points_list) > 2:
                for point in mission_2_points_list:  # 列表中的第三个一定是目标点
                    count += 1
                    logger.info(f"move to {point}")
                    navi.direct_set_waypoint(point)
                    navi.wait_for_height(time_thres=0.05, timeout=10)
                    navi.wait_for_waypoint(
                        time_thres=1.5, pos_thres=7, timeout=10)
                    self.next_point_event.clear()
                    self.identify_once()
                    self.next_point_event.wait()
                    self.next_point_event.clear()
                    res = self.QR_res
                    if count == 3:
                        self.report(id, res)
                        self.gpio_set_laser(side, 1)
                        time.sleep(0.8)
                        self.gpio_set_laser(side, 0)
            else:
                self.stop()
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
