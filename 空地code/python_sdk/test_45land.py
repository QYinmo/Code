
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
cfg = ConfigManager(section="mission")
SPEED=15
HEIGHT=85
BASE_POINT=[0,0]
MISSION_POINTS: np.ndarray = np.array([(0, 0, HEIGHT), (0, -100, 10)])
BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
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
TRAJ_LENGTH += random.randint(-100, 100) / 10



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
        self.ob_count = 0  # 确认有
        self.res_count = 0  # 确认对
        self.timeout = 0  # 超时
        self.identify_status = False  # 识别状态
        self.last_res = -1
        self.started = False
    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")
    def land_45(self,land_point=(100,0,105)):
        DT=0.1
        navi=self.navi
        navi.navigation_to_waypoint(land_point,wait=True)
        land_path=np.array([
            land_point,(0,0,10)
        ])
        plan_path=[]
        for i in range(len(land_path) - 1):
            last_p = land_path[i]
            next_p = land_path[(i + 1)]

            # 计算两点之间的距离
            length = np.linalg.norm(next_p - last_p)

            # 设定点之间的间距为5cm (0.05米)
            spacing = 5 
            
            # 计算需要插入的点的数量，向上取整
            num_points = int(np.ceil(length / spacing))

            # 在两点之间进行线性插值
            for j in range(num_points + 1):
                # 计算插值参数 t
                t = j / num_points
                
                # 计算新的点坐标
                new_point = last_p + t * (next_p - last_p)
                
                # 将新点添加到路径中
                plan_path.append(new_point)
        for _ in range(3):
            time.sleep(0.25)
            fc.set_indicator_led(255, 0, 0) 
            time.sleep(0.25)
            fc.set_indicator_led(0, 0, 0)      
        navi.navigation_follow_trajectory(np.array(plan_path),wait=True,dt=0.15)
        #定点降落（改）
        navi.navigation_flag = True
        navi.keep_height_flag = True
        navi.switch_pid("land")
        time.sleep(0.5)
        navi.set_height(10)
        time.sleep(1)
        navi.wait_for_height(timeout=3)
        navi.set_height(0)
        time.sleep(2)
        navi.fc.lock()
        navi.navigation_flag = False
        navi.keep_height_flag = False        
    def run(self):
        fc = self.fc
        radar = self.radar
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
        ################  校准 ################
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(False)
        fc.set_indicator_led(0, 255, 0)
        time.sleep(0.25)        
        self.started = True
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")
        time.sleep(0.8)
        ################ 初始化完成 ################
        for _ in range(3):
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(1)
            fc.set_indicator_led(0, 0, 0)  # 起飞前警告
            time.sleep(1)
        self.navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        time.sleep(0.2)
        self.navi.wait_for_height()
        time.sleep(2)
        self.navi.pointing_landing(navi.current_point)



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
        logger.debug(f"{Exception}")
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