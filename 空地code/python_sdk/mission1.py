"""
使用雷达作为位置闭环的任务模板
"""
import random
import struct
import threading
import time
from typing import List

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
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger
from trash.vision_fire import get_fire_loc,find_fire
from trash.pid import pid
from trash.usr_serial import Serial_car, Serial_gpio

def deg_360_180(deg):
    if deg > 180:
        deg = deg - 360
    return deg


cfg = ConfigManager(section="mission")

BASE_CALI_POINT = cfg.get_array(
    "point-base", default=np.array([81.28021476, 418.3601995]))

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
SPEED = 25
HEIGHT = 180
P = lambda x, y: np.array([x * 72+5, -y * 72-10, HEIGHT])


mission_points_list = []
mission_points_list.append(P(0,5))
mission_points_list.append(P(1,5))
mission_points_list.append(P(1,0))
mission_points_list.append(P(2,0))
mission_points_list.append(P(2,5))
mission_points_list.append(P(3,5))
mission_points_list.append(P(3,0))
mission_points_list.append(P(4,0))
mission_points_list.append(P(4,5))

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
        self.started = False
        self.fire_running = False
        self.found_fire = False
        self.fire_x = 0
        self.fire_y = 0
        self.pid=None
        self.mode=1
        self.timecount0=0
        self.flag0=0
        self.flag1=0
        self.car_com=[170,0,0,0,0,255]
        self.gpio_com=[170, 0, 0, 0, 0, 0,1, 0, 0, 255]
        self.car_rxbuffer=[0,0]
        self.serial_car=Serial_car(port="/dev/ttyUSB1",baudrate=115200,rx_length=4)
        self.serial_gpio=Serial_gpio(port="/dev/ttyUSB2",baudrate=38400)
        if self.fc.last_command_done:
            pass

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def start_find_fire(self):
        self.fire_running = True
        threading.Thread(target=self.find_fire_task, daemon=True).start()
    def gpio_set_laser(self,switch):
        self.gpio_com[6]=switch
    def gpio_drop_package(self):
        self.gpio_com[7]=31 ##经测试 范围为E1~21 E1为完全收拢 21为完全张开
    def find_fire_task(self):
        time.sleep(8)
        while self.fire_running:
            if not self.found_fire:
                res = find_fire(self.cam.read()[1])
                if res is not None:
                    logger.info("Fire found")
                    self.navi.navigation_stop_here()
                    self.found_fire = True
                    self.pid=pid([res[0],res[1]])
                    self.navi.navigation_flag=False
                    self.fc.send_realtime_control_data(vel_x=int(0),vel_y=int(0))
            else:
                res=get_fire_loc(self.cam.read()[1])
                if res is None:
                    logger.warning("Fire lost")
                    continue
                else:

                    sp=self.move_to_fire(res)
                    if abs(sp[0])<8 and abs(sp[1])<8:   
                        if self.flag0==0:
                            self.timecount0=time.time()
                            self.flag0=1
                            break
            time.sleep(0.02)
        self.put_package()
        logger.info("Fire put down")
            
    def move_to_fire(self, current):

        speed=self.pid.get_cv_pid(current)
        self.fc.send_realtime_control_data(vel_x=int(speed[0]),vel_y=int(speed[1]))
        logger.info(f"speed:{speed}")
        return speed

    def put_package(self):
        self.fire_running=False
        self.navi.navi_x_pid.setpoint = self.navi.current_x
        self.navi.navi_y_pid.setpoint = self.navi.current_y
        self.car_com[3]=abs(int(self.navi.current_x/5))
        self.car_com[4]=abs(int(self.navi.current_y/5))
        self.navi.navigation_flag=True
        self.gpio_set_laser(1)
        self.navi.set_height(80)
        logger.info("set height1")
        time.sleep(4)  
        self.navi.set_height(50)
        self.gpio_drop_package()
        time.sleep(8)
        self.navi.set_height(HEIGHT)
        logger.info("set height2")
        time.sleep(8)
        # self.fc.send_realtime_control_data(vel_x=int(0),vel_y=int(0))
        logger.info("navi restart")
        self.flag1=1
        return
    def rx_scan_task(self):
        while True:
            if self.car_rxbuffer[1] is 1 :
                self.mode=0
                #logger.info("FIRE MODE")
            else:
                self.mode=1
            if self.car_rxbuffer[0] is not 0:
                self.takeoff_event.set()
                logger.info("TAKE OFF SIGNAL RECEIVED")
                logger.info(f"mission {self.mode+1} start")
                break;
            time.sleep(0.1)
    def tx_change_task(self):
        while True:
            self.car_com[1]=abs(int(self.navi.current_x/5))
            self.car_com[2]=abs(int(self.navi.current_y/5))
            time.sleep(0.05)
            
    def start_rx_scan_task(self):
        threading.Thread(target=self.rx_scan_task, daemon=True).start()
    def start_tx_change_task(self):
        threading.Thread(target=self.tx_change_task, daemon=True).start()
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
        self.serial_car.start_transmit(self.car_com,self.car_rxbuffer)
        self.serial_gpio.send_start(self.gpio_com)
        self.start_rx_scan_task()
        self.start_tx_change_task()
        ################  校准 ################
        navi.set_basepoint(BASE_POINT)
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(False)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480.0)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cam.set(cv2.CAP_PROP_AUTO_EXPOSURE,1)
        cam.set(cv2.CAP_PROP_EXPOSURE, 35)
        self.gpio_set_laser(0)
        time.sleep(0.25)
        self.takeoff_event.clear()
        self.takeoff_event.wait()
        self.takeoff_event.clear()
        self.started = True
        for _ in range(3):
            time.sleep(0.5)
            time.sleep(0.5)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")
        ################ 初始化完成 ################
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        if self.mode==1:
            self.start_find_fire()
        navi.navigation_follow_trajectory(MISSION_TRAJ, wait=True)
        while not self.flag1:
            time.sleep(0.1)
        navi.navigation_follow_trajectory(self.navi.traj_list_before_stop, wait=True)
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
