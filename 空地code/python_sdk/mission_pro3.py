"""
消防小车和无人机空地协同题
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


cfg = ConfigManager(section="mission")

BASE_CALI_POINT = cfg.get_array(
    "point-base", default=np.array([81.28021476, 418.3601995]))

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
SPEED = 25
HEIGHT = 170
def P(x, y): return np.array([x * 80 + 5, -y * 80 - 5, HEIGHT])


MISSION_POINTS: np.ndarray = np.array(
    [
        P(0, 0),
        P(0, 1),
        P(3, 1),
        P(3, 2),
        P(0, 2),
        P(0, 3),
        P(3, 3),
        P(3, 4),
        P(0, 4),
        P(0, 5),
        P(4, 5),
        P(4, 0),
        P(0, 0),
    ]
)
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

        self.fire_time = 0
        self.started = False
        self.found_fire = False
        self.fire_x = 0
        self.fire_y = 0
        self.takeoff_flag = False
        self.use_serial = True
        self.gpio_com = [170, 0, 0, 0, 255]  # 镭射，蜂鸣器，电磁铁
        # 无人机位置x，y，里程mileage，火源位置x，y，火源街道，火源片区
        self.terminal_com = [170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255]
        self.rxbuffer = [0]
        if self.use_serial:
            self.serial_terminal = Serial_station(
                device="cp2102", baudrate=115200, rx_length=3)
            self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)
        if self.fc.last_command_done:
            pass

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def laser(self, status):
        self.gpio_com[1] = status
        if status:
            logger.info("[MISSION] Laser ON")
        else:
            logger.info("[MISSION] Laser OFF")

    def Buzzer(self):
        self.gpio_com[2] = 1
        time.sleep(0.5)
        self.gpio_com[2] = 0
        time.sleep(0.5)
        self.gpio_com[2] = 1
        time.sleep(0.5)
        self.gpio_com[2] = 0
        time.sleep(0.5)
        self.gpio_com[2] = 1
        time.sleep(0.5)
        self.gpio_com[2] = 0
        time.sleep(0.5)
        # 蜂鸣

    def electromagnet(self, state):
        self.gpio_com[3] = state
        logger.info("[MISSION] Electromagnet mission")

    def update_terminal_data(self):
        while True:
            self.terminal_com[1] = round(abs(self.navi.current_x)/2)
            self.terminal_com[2] = round(abs(self.navi.current_y)/2)
            time.sleep(0.2)

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
        # self.serial_terminal.listen_start(self.rxbuffer)
        self.serial_gpio.send_start(self.gpio_com)
        time.sleep(0.1)

        logger.info("[MISSION] Waiting for takeoff command...")
        self.rxbuffer[0] = 1
 #       while not self.takeoff_flag:
    #        logger.info(f"[MISSION] Received data: {self.rxbuffer[1]}")
     #       if self.rxbuffer[0] == 1:
        #          self.takeoff_flag = True
       #         self.serial_terminal.listen_pause()
        #    time.sleep(0.1)
        self.serial_terminal.send_start(self.terminal_com)
   #             logger.info("[MISSION] Takeoff command received")
        time.sleep(0.03)
        self.data_thread = threading.Thread(
            target=self.update_terminal_data, daemon=True)
        time.sleep(0.03)
        self.data_thread.start()
        time.sleep(0.1)

        self.started = True

        self.gpio_com[3] = 1
        self.gpio_com[1] = 1
        for _ in range(3):
            time.sleep(0.25)
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(0.25)
            fc.set_indicator_led(0, 0, 0)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")
        ################ 初始化完成 ################
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        navi.navigation_follow_trajectory(MISSION_TRAJ, wait=False)
        self.check_fire()
        navi.pointing_landing(LANDING_POINT)

    def check_fire(self):
        while True:
            _, frame = self.cam.read()
            if frame is None:
                logger.warning("NO PICTURE")
                continue
            img = get_ROI(frame, (0.3, 0.3, 0.4, 0.4))
            f, dx, dy, _ = find_red_area(img)
            debug_imshow(frame, "Origin")
            if f:
                f, _, _, area = find_red_area(frame)
                if f and area < 8000:
                    self.fire_time += 1
                    if self.fire_time > 1:
                        self.fire_time = 0
                        self.send_restart = 1
                        logger.info("停下导航")
                        self.navi.navigation_stop_here()
                        logger.info(f"[MISSION] Fire found at {dx}, {dy}")
                        self.drop_pack()
                        logger.info(
                            "[MISSION] Pack dropped, continue trajectory")
                        self.navi.navigation_follow_trajectory(
                            self.navi.traj_list_before_stop, wait=False)
                        break
            else:
                self.fire_time = 0
            if not self.navi.traj_running_event.is_set():
                logger.info("[MISSION] Trajectory finished, no fire found")
                break
            time.sleep(0.08)

    def drop_pack(self):
        K = 2
        time_found = 0
        state = 0
        navi = self.navi
        fc = self.fc
        fc.set_rgb_led(255, 255, 0)
        self.send_running = False
        while True:
            _, frame = self.cam.read()
            if frame is None:
                continue
            img = get_ROI(frame, (0.27, 0.27, 0.46, 0.46))
            point = navi.current_point
            f, dx, dy, area = find_red_area(img)
            if f:
                if state == 0 and abs(dx) < 15 and abs(dy) < 15:
                    logger.info(f"[MISSION] Reached fire at {point}")

                    navi.set_height(100)
                    state = 1
                if state == 1 and abs(navi.current_height - navi.height_pid.setpoint) < 8:
                    logger.info(f"[MISSION] Reached height at {point}")
                    time_found = time.perf_counter()
                    fc.set_rgb_led(255, 0, 0)
                    state = 2
                if state == 2 and time.perf_counter() - time_found > 3:
                    logger.info(f"[MISSION] Released pack at {point}")
                    self.electromagnet(0)
                    self.fire_x = point[0]
                    self.fire_y = point[1]
                    self.found_fire = 1
                    self.terminal_com[4] = self.found_fire
                    self.terminal_com[5] = round(abs(self.fire_x)/2)
                    self.terminal_com[6] = round(abs(self.fire_y)/2)

                    fc.set_rgb_led(0, 0, 0)
                    navi.set_height(self.cruise_height)
                    navi.wait_for_height()
                    self.restart = 1
                    self.send_running = True
                    return
                to_point = (point[0] - dy / K, point[1] - dx / K)
                navi.direct_set_waypoint(to_point)
            time.sleep(0.08)

    def fire_side(self, x, y, n):
        dx = x - self.fire_x
        dy = y - self.fire_y
        if n == 4 or n == 2:
            if dx >= 0:
                return 1
            else:
                return 3
        elif n == 6:
            if dy >= 0:
                return 2
            else:
                return 4
        else:
            if dx >= 0:
                return 1
            else:
                return 3


if __name__ == "__main__":
    rm = RosManager()
    rm.chmod("/dev/ttyUSB0")
    rm.chmod("/dev/ttyACM1")
    rm.chmod("/dev/video1")
    rm.launch_package("ldlidar_stl_ros2", "ld06.launch.py")
    rm.launch_package("realsense2_camera", "rs_launch.py")
    rm.launch_package("cartographer_ros", "cartographer.launch.py")
    rm.run_package("tf2_ros", "static_transform_publisher",
                   "0 0 0 0 0 0 camera_pose_frame base_link")
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
