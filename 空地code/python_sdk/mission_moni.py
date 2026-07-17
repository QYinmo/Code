import random
import struct
import threading
import time
from typing import List
from FlightController.Components.RosManager import RosManager
import cv2
import numpy as np
from simple_pid import PID
from config_manager import ConfigManager
from FlightController import FC_Client, FC_Controller, FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Components.UartScreen import UARTScreen
from FlightController.Solutions.Navigation import Navigation
from FlightController.Solutions.PathPlanner import PFBPP, TrajectoryGenerator, AStar, ObstacleGenerator
from vision.Vision_plus import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger
from usr_serial import Serial_station, Serial_gpio
from ultralytics import YOLO
timer = HighPrecisionFPS()
cfg = ConfigManager(section="mission")

BASE_CALI_POINT = cfg.get_array(
    "point-base", default=np.array([81.28021476, 418.3601995]))

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
MISSION_POINTS: np.ndarray = np.array([])

SPEED = 20
HEIGHT = 130

A_point = np.array([120, -130])
B_point = np.array([120, -230])


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.rs: T265 = kwargs["rs"]
        self.navi: Navigation = kwargs["navi"]
        self.mode = 0
        self.use_serial = True
        self.started = False
        self.conf_thres = 0.9
        self.identify_status = False
        self.area_count = 0
        self.not_see = True
        self.area_timeout = 0
        self.area_res = -1
        self.hover = False  # 是否开始盘旋
        self.gpio_com = [170, 0, 0, 0, 255]  # 激光笔，上面的舵机，下面的舵机
        self.terminal_com = [170, 0, 0, 0, 0, 255]
        self.terminal_rxbuffer = [0, 0, 0]
        if self.use_serial:
            self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)
        if self.fc.last_command_done:
            pass
        self.ob_count = 0  # 确认有
        self.res_count = 0  # 确认对
        self.timeout = 0  # 超时
        self.last_res = -1
        self.started = False
        self.res = -1
        self.already = np.array([])  # 记录已经查询过的点
        self.scope = 10  # 在多少范围内认为是同一个点
        self.allow_repeat = True  # 是否不需要查重
        self.use_pid = True  # 是否pid到位置#暂时都会用pid
        self.choose_point = 0  # 选择点a或b
        self.mission_point = np.array([])  # 任务点

    def circle(self, center_x, center_y, radius, num_points=50):
        """
        :param center_x: 圆心X坐标 (cm)
        :param center_y: 圆心Y坐标 (cm)
        :param radius: 巡逻半径 (cm)
        :param num_points: 圆周上轨迹点的数量，点越多轨迹越平滑
        """
        logger.info(f"在 ({center_x}, {center_y}) 周围进行半径 {radius}cm 的圆形巡逻...")

        trajectory_points = []
        for i in range(num_points + 1):  # +1 确保回到起点
            angle_rad = np.deg2rad(i * (360 / num_points))
            x = center_x + radius * np.sin(angle_rad)
            y = center_y - radius * np.cos(angle_rad)
            trajectory_points.append((x, y, HEIGHT))
        return trajectory_points

    def show_degree(self, point):
        while True:
            if point == 0:
                x = navi.current_point[0]-A_point[0]
                y = A_point[1]-navi.current_point[1]
                degree = np.arctan2(x, y) * 180 / np.pi
            else:
                x = navi.current_point[0]-B_point[0]
                y =B_point[1]- navi.current_point[1]
                degree = np.arctan2(x, y) * 180 / np.pi
            if self.count>20:    
                if degree < 0:
                    degree += 360        
            logger.info(f"当前角度: {degree:.2f}°")
            p_degree=degree/360.0
            if p_degree < 0:
                self.gpio_com[2] = 0
            elif p_degree > 0:
                self.gpio_com[2] = round(p_degree*200)
            self.count+=1
            time.sleep(0.3)

    def start_show_degree(self, point):
        threading.Thread(target=self.show_degree,
                         args=(point,), daemon=True).start()

    def id_function(self, img):
        debug_imshow(img, "Origin")
        logger.debug("正在识别")
        f, dx, dy, area = find_largest_contour_info(img)
        if f:
            logger.info(f"area:{area}")
            return True, dx, dy, area
        return False, 0, 0, 0

    def servo1(self, state):  # 0不动1动
        self.gpio_com[2] = state

    def servo2(self, state):  # 0往下转2往上转1不动
        self.gpio_com[3] = state

    def target(self):
        logger.info("找到东西，开始瞄准")
        datapre = 2000
        while True:
            ret, img = self.cam.read()
            self.servo1(0)
            self.servo2(1)
            if not ret:
                logger.warning("没有图像")
                continue
            img = get_ROI(img, (0.27, 0, 0.46, 1))
            f, dx, dy, data = self.id_function(img)
            if abs(data-datapre) > 1000:
                datapre = data
                continue
            if f and data > 2000:
                if dx > 200:
                    logger.info("快了")
                    self.servo1(0)
                elif dx < -200:
                    logger.info("慢了")
                    self.servo1(2)
                else:
                    logger.info("x_good")
                    self.servo1(1)
                if dy > 150:
                    logger.info("近了")
                    self.servo2(2)
                elif dy < -150:
                    logger.info("远了")
                    self.servo2(0)
                else:
                    logger.info("y_good")
                    self.servo2(1)
            else:
                logger.warning("跟丢了")
            if not self.hover:
                break
            time.sleep(0.08)

    def do_task(self):
        # 写识别到某结果后的任务
        if self.res == 1:
            logger.info("这是")

    def camera_task(self):
        self.ob_count = 0
        self.timeout = 0
        navi = self.navi
        while True:
            ret, img = self.cam.read()
            if not ret:
                logger.warning("没有图像")
                continue
            if self.identify_status:
                f, _, _, res = self.id_function(img)
                if f and res > 2000:
                    self.ob_count += 1
                else:
                    self.ob_count = 0
                    logger.debug("没找到框")
                if not self.allow_repeat:
                    curpoint = navi.current_point
                    if len(self.already) > 0:
                        # 计算所有点与新点的欧氏距离
                        distances = np.linalg.norm(
                            self.already - curpoint, axis=1)
                        if np.any(distances < self.scope):
                            self.ob_count = 0
                if self.ob_count > 1:  # 此处改确认存在次数
                    self.ob_count = 0
                    self.timeout = 0
                    self.identify_status = False
                    if self.started:
                        time.sleep(0.1)
                        self.res = int(res)
                        self.target()
            time.sleep(0.06)
            print(f"瞬时FPS: {timer.fps():.1f}")
            timer.reset()

    def init_point(self, choose):
        if choose == 0:
            cx, cy = A_point
        elif choose == 1:
            cx, cy = B_point
        mission_points = self.circle(cx, cy, 100, num_points=100)
        self.mission_point = np.array(mission_points)
        logger.info(f"生成任务点: {mission_points}")

    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def run(self):
        fc = self.fc
        cam = self.cam
        navi = self.navi

        self.navigation_speed = SPEED
        self.cruise_height = HEIGHT
        self.vertical_speed = 15
        set_manual_exporsure(cam, -8)
        self.inital_yaw = self.fc.state.yaw.value
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()
        navi.switch_navigation_mode("fusion-ros")
        navi.set_basepoint(BASE_POINT)
        navi.set_rs_speed_report(True, 4)
        fc.set_action_log(False)
        # self.start_camera_task()
        vision_debug(True)
        fc.set_indicator_led(0, 255, 0)
        time.sleep(0.05)
        self.serial_gpio.send_start(self.gpio_com)
        time.sleep(0.1)
        self.gpio_com[1] = 0
        self.gpio_com[2] = 0
        self.gpio_com[3] = 0
        self.started = True
        for _ in range(3):
            time.sleep(0.25)
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(0.25)
            fc.set_indicator_led(0, 0, 0)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")

        self.init_point(self.choose_point)

        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        time.sleep(0.5)

        navi.navigation_to_waypoint(A_point, wait=True)
        time.sleep(3)
        navi.navigation_to_waypoint(B_point, wait=True)
        time.sleep(3)
        self.identify_status = False
        self.hover = True
        time.sleep(1)
        navi.navigation_to_waypoint(self.mission_point[0], wait=True)
        navi.set_navigation_speed(15)
        self.gpio_com[1] = 1
        self.start_show_degree(self.choose_point)
        navi.navigation_follow_trajectory(self.mission_point, wait=False)

        while True:
            if not navi.traj_running_event.is_set():
                logger.info("[MISSION] Trajectory completed")
                self.gpio_com[2] = 0
                break
            time.sleep(0.2)
        logger.info("[MISSION] Mission Completed")
        time.sleep(1)
        self.gpio_com[1] = 0
        navi.navigation_to_waypoint(BASE_POINT, wait=True)
        fc.set_rgb_led(0, 255, 0)


if __name__ == "__main__":
    fc = FC_Client()
    fc.connect()
    fc.wait_for_connection()
    t265 = T265("ros")
    t265.start()
    radar = LD_Radar()
    radar.start("ros")
    cam = open_camera_plus()
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
