"""
使用雷达作为位置闭环的任务模板
"""
import threading
import time
from cgitb import text
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
from FlightController.Solutions.PathPlanner import PFBPP
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger


def deg_360_180(deg):
    if deg > 180:
        deg = deg - 360
    return deg


cfg = ConfigManager(section="mission")

BASE_CALI_POINT = cfg.get_array("point-base", default=np.array([81.28021476, 418.3601995]))

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
MISSION_POINTS: np.ndarray = np.array([[266, 6], [278, 178], [252, 350], [142, 196], [15, 333]])


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.rs: T265 = kwargs["rs"]
        self.navi: Navigation = kwargs["navi"]
        self.screen: UARTScreen = kwargs.get("screen", None)

        self.list_vip1 = []
        self.list_vip2 = []
        self.list_vip3 = []
        self.pending_list_vip1 = []
        self.pending_list_vip2 = []
        self.pending_list_vip3 = []
        self.land_event = threading.Event()
        self.ok_event = threading.Event()
        self.emergency_event = threading.Event()
        self.takeoff_event = threading.Event()

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def beep(self):
        self.fc.set_rgb_led(0, 255, 0)
        self.fc.set_digital_output(1, True)
        time.sleep(0.2)
        self.fc.set_digital_output(1, False)
        self.fc.set_rgb_led(0, 0, 0)

    def callback(self, data: str):
        try:
            if data.startswith("add"):
                point = int(data[3])
                emergency = int(data[4])
                if emergency == 0:
                    self.pending_list_vip1.append(point)
                elif emergency == 1:
                    self.pending_list_vip2.append(point)
                elif emergency == 2:
                    self.pending_list_vip3.append(point)
                    self.emergency_event.set()
                self.log_list(f"新订单已收到(目标:{point+1}, 紧急:{emergency+1})")
            elif data.startswith("land"):
                self.land_event.set()
                self.screen.set_widget_value("button_land.visible", 0)
            elif data.startswith("ok"):
                self.ok_event.set()
                self.screen.set_widget_value("button_ok.visible", 0)
            elif data.startswith("takeoff"):
                self.takeoff_event.set()
                self.screen.set_widget_value("button_takeoff.visible", 0)
            threading.Thread(target=self.beep, daemon=True).start()
        except Exception as e:
            logger.error(e)

    def run(self):
        fc = self.fc
        radar = self.radar
        cam = self.cam
        navi = self.navi
        ############### 参数 #################
        self.camera_down_pwm = 65
        self.camera_up_pwm = 28
        self.navigation_speed = 30  # 导航速度
        self.cruise_height = 140  # 巡航高度
        self.vertical_speed = 20  # 垂直速度
        ################ 启动线程 ################
        self.inital_yaw = self.fc.state.yaw.value
        # self.fd = FastestDetOnnx(drawOutput=True)  # 初始化神经网络
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()  # 启动导航线程
        navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started")
        self.log("无人机初始化完成")
        ################  校准 ################
        navi.set_basepoint(BASE_CALI_POINT)
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(False)
        change_cam_resolution(cam, 640, 480, 60)
        set_cam_autowb(cam, True)
        fc.set_PWM_output(0, self.camera_down_pwm)
        fc.set_indicator_led(0, 255, 0)
        fc.event.key_short.clear()
        fc.event.key_short.wait_clear()
        fc.set_digital_output(0, False)  # 激光笔
        fc.set_digital_output(2, False)  # 激光笔
        fc.set_action_log(True)
        ################ 初始化完成 ################
        logger.info("[MISSION] Mission Started")
        self.log("正在建图, 请稍后")
        fc.set_indicator_led(255, 255, 0)
        self.init_bfp()
        while True:
            fc.set_digital_output(1, True)
            fc.set_indicator_led(0, 255, 0)
            time.sleep(0.25)
            fc.set_digital_output(1, False)
            while len(self.list_vip1) == 0 and len(self.list_vip2) == 0 and len(self.list_vip3) == 0:
                self.log("无人机状态空闲, 可以接受订单")
                time.sleep(1)
            self.ready_for_takeoff()
            self.add_pending_list()
            now_loc = BASE_POINT
            while self.have_target():
                target, emergency = self.get_one_target()
                self.log_list(f"正在前往配送点-{target+1}")
                loc = MISSION_POINTS[target]
                path_go = self.calc_bfp(now_loc, loc)
                self.takeoff(now_loc)
                navi.navigation_follow_trajectory(path_go, wait=False)
                new_order = False
                while navi.traj_running_event.is_set():
                    time.sleep(0.1)
                    if self.have_pending():  # 有新订单, 回基地
                        navi.navigation_stop_here()
                        self.log_list("正在返回基地处理新订单")
                        self.insert_one_target_back(target, emergency)
                        self.back_to_base()
                        self.add_pending_list()
                        now_loc = BASE_POINT
                        new_order = True
                        break
                if new_order:
                    continue
                self.ready_for_land()
                navi.pointing_landing(loc)
                now_loc = loc
                self.ready_for_ok()
                time.sleep(1)
            self.log("所有订单已完成, 正在返回基地")
            self.back_to_base()

    def back_to_base(self):
        path_back = self.calc_bfp(self.navi.current_point, BASE_POINT)
        self.navi.navigation_follow_trajectory(path_back, wait=True)
        self.navi.pointing_landing(BASE_POINT)
        time.sleep(1)

    def add_pending_list(self):
        while len(self.pending_list_vip3) > 0:
            self.list_vip3.append(self.list_vip3.pop(0))
        while len(self.pending_list_vip2) > 0:
            self.list_vip2.append(self.list_vip2.pop(0))
        while len(self.pending_list_vip1) > 0:
            self.list_vip1.append(self.list_vip1.pop(0))

    def have_pending(self):
        return len(self.pending_list_vip1) + len(self.pending_list_vip2) + len(self.pending_list_vip3) > 0

    def get_one_target(self):
        if len(self.list_vip3) > 0:
            return self.list_vip3.pop(0)
        elif len(self.list_vip2) > 0:
            return self.list_vip2.pop(0)
        elif len(self.list_vip1) > 0:
            return self.list_vip1.pop(0)
        else:
            return -1, -1

    def insert_one_target_back(self, target, emergency):
        if emergency == 3:
            self.pending_list_vip3.insert(0, target)
        elif emergency == 2:
            self.pending_list_vip2.insert(0, target)
        elif emergency == 1:
            self.pending_list_vip1.insert(0, target)

    def have_target(self):
        return len(self.list_vip1) + len(self.list_vip2) + len(self.list_vip3) > 0

    def log(self, msg):
        self.screen.set_widget_value("text_log.txt", msg)

    def log_list(self, msg):
        text = f"{msg}"
        if self.have_target:
            text += f"\n排队队列:{self.list_vip1+self.list_vip2+self.list_vip3}"
        self.screen.set_widget_value("text_log.txt", text)

    def ready_for_land(self):
        self.log_list("已到达配送点, 请许可降落")
        self.screen.set_widget_value("button_land.visible", 1)
        state = False
        if not self.land_event.wait(0.1):
            state = not state
            self.fc.set_digital_output(2, state)
            self.fc.set_digital_output(1, state)
        self.fc.set_digital_output(2, False)
        self.fc.set_digital_output(1, False)
        self.land_event.clear()
        self.log_list("许可收到, 正在降落!")

    def ready_for_ok(self):
        self.log_list("订单已送达, 请确认收货")
        self.screen.set_widget_value("button_ok.visible", 1)
        state = False
        if not self.ok_event.wait(0.1):
            state = not state
            self.fc.set_digital_output(0, state)
            self.fc.set_digital_output(1, state)
        self.fc.set_digital_output(0, False)
        self.fc.set_digital_output(1, False)
        self.ok_event.clear()
        self.log_list("订单已完成, 请给五星好评!")

    def ready_for_takeoff(self):
        self.screen.set_widget_value("button_takeoff.visible", 1)
        self.takeoff_event.wait()
        self.takeoff_event.clear()

    def takeoff(self, point):
        for _ in range(3):
            time.sleep(0.25)
            self.fc.set_digital_output(1, True)  # 蜂鸣器
            self.fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(0.25)
            self.fc.set_digital_output(1, False)  # 蜂鸣器
            self.fc.set_indicator_led(0, 0, 0)
        self.navi.pointing_takeoff(point, self.cruise_height)

    def init_bfp(self):
        point = self.radar.map.find_nearest(0, 90, 1)[0].to_xy() / 1000
        # point = np.array([-point[1], -point[0]])
        loc = point + np.array([-0.05, 0.05])
        loc2 = loc + np.array([1.1, -1.1])
        logger.debug(f"loc: {loc}, loc2: {loc2}")
        start_point = (0.0, 0.0)
        goal_point = MISSION_POINTS[0]
        grid_size = 0.1
        robot_radius = 0.5
        map_width = 5.0
        bfp = PFBPP()
        obstacle = []
        for x in np.arange(loc[0], loc2[0], 0.1 if loc[0] < loc2[0] else -0.1):
            obstacle.append((x, loc[1]))
            obstacle.append((x, loc2[1]))
        for y in np.arange(loc[1], loc2[1], 0.1 if loc[1] < loc2[1] else -0.1):
            obstacle.append((loc[0], y))
            obstacle.append((loc2[0], y))
        bfp.set_plan_path(start_point, goal_point)
        bfp.set_params(map_width, grid_size, robot_radius)
        bfp.set_obstacle(obstacle)
        bfp.calc_potential_field()
        self.bfp = bfp

    def calc_bfp(self, from_point, to_point):
        from_point = np.array(from_point) / 100
        to_point = np.array(to_point) / 100
        self.bfp.set_plan_path((from_point[0], from_point[1]), (to_point[0], to_point[1]))
        self.bfp.calc_potential_field()
        path = self.bfp.run_planner()
        return np.array(path) * 100


if __name__ == "__main__":
    logger.warning("DEBUG MODE!!")
    fc = FC_Controller()
    fc.start_listen_serial(print_state=True)
    fc.wait_for_connection()
    # fc = FC_Client()
    # fc.connect()
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
