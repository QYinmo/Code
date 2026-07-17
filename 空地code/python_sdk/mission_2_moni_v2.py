"""
使用雷达作为位置闭环的任务模板
"""
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
from loguru import logger
from FlightController.Components.RosManager import RosManager 
from ultralytics import YOLO

bi = 2
ci = 1

cfg = ConfigManager(section="mission")

BASE_CALI_POINT = cfg.get_array("point-base", default=np.array([81.28021476, 418.3601995]))

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
SPEED = 25
HEIGHT = 150
MISSION_POINTS = [
    (70,-70,HEIGHT), # 0
    (70,-160,HEIGHT), # 1
    (70,-250,HEIGHT), # 2
    (70,-340,HEIGHT), # 3
    (160,-340,HEIGHT), # 4
    (250,-340,HEIGHT), # 5
    (250,-250,HEIGHT), # 6
    (160,-250,HEIGHT), # 7
    (160,-160,HEIGHT), # 8
    (250,-160,HEIGHT), # 9
    (250,-70,HEIGHT), # 10
    (160,-70,HEIGHT), # 11
]


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.rs: T265 = kwargs["rs"]
        self.navi: Navigation = kwargs["navi"]
        self.screen: UARTScreen = kwargs.get("screen", None)
        self.takeoff_event = threading.Event()
        self.cnt = 0 # 无人机位置计数
        self.started = False
        self.location_array = [
            False,False,False,False,False,False,False,False,False,False,False,False
        ]
        # 水源位置
        self.lake_point = None
        # 火源位置
        self.wildfire_point = None
        # 泥石流位置
        self.debris_point = None
        # YOLOV8
        self.check()

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def send_wireless(self):
        while True:
            time.sleep(0.2)
            data = struct.pack(
                "<BBBBBBBBBBBBB",
                int(self.started),
                int(self.location_array[0]),
                int(self.location_array[1]),
                int(self.location_array[2]),
                int(self.location_array[3]),
                int(self.location_array[4]),
                int(self.location_array[5]),
                int(self.location_array[6]),
                int(self.location_array[7]),
                int(self.location_array[8]),
                int(self.location_array[9]),
                int(self.location_array[10]),
                int(self.location_array[11])
            )
            self.fc.send_to_wireless(data)

    def receive_wireless(self, data: bytes):
        action = struct.unpack("<B", data[0:1])[0]
        if action == 1:
            self.takeoff_event.set()

    def bibi(self):
        fc = self.fc
        fc.set_digital_output(bi,True)
        time.sleep(0.4)
        fc.set_digital_output(bi,False)
        time.sleep(0.4)
        fc.set_digital_output(bi,True)
        time.sleep(0.4)
        fc.set_digital_output(bi,False)
        time.sleep(0.4)
        fc.set_digital_output(bi,True)
        time.sleep(0.4)
        fc.set_digital_output(bi,False)
        time.sleep(0.4)

    def run(self):
        fc = self.fc
        radar = self.radar
        navi = self.navi
        ############### 参数 #################
        self.navigation_speed = SPEED  # 导航速度
        self.cruise_height = HEIGHT  # 巡航高度
        self.vertical_speed = 20  # 垂直速度
        cam = self.cam
        ################ 启动线程 ################
        self.inital_yaw = self.fc.state.yaw.value
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()  # 启动导航线程
        navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started")
        threading.Thread(target=self.send_wireless, daemon=True).start()
        fc.register_wireless_callback(self.receive_wireless)
        ################  校准 ################
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(False)
        fc.set_indicator_led(0, 255, 0)
        fc.set_digital_output(ci,True)
        time.sleep(0.25)
        self.takeoff_event.clear()
        self.started = True
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")
        ################ 初始化完成 ################
        while True:
            time.sleep(1)
            logger.info(f"current_point: {navi.current_point}")
            if navi.current_point[0] + navi.current_point[1] != 0:
                break
        fc.set_indicator_led(0, 0, 0)
        self.bibi() # 起飞前警告
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        for self.cnt in range(len(MISSION_POINTS)):
            navi.navigation_to_waypoint(MISSION_POINTS[self.cnt])
            fc.set_indicator_led(0,0,255) # 蓝灯提示
            self.check()
            fc.set_indicator_led(0,0,0) # 关灯
        navi.pointing_landing(LANDING_POINT)

    def check(self):
        model = YOLO('YOLOV8/weights/best_int8_openvino_model/',task='detect')
        cap, i = open_camera()
        logger.info(f"Camera {i} opened")
        change_cam_resolution(cap, 640, 480, 30)
        set_cam_autowb(cap, True)
        while True:
            time.sleep(0.01)
            frame = cap.read()[1]
            if frame is not None:
                break
        img = get_ROI(frame,((0.3, 0.3, 0.47, 0.6)))
        results = model(source=img, show=False, conf=0.7, save=False, verbose=False, stream=True, device='cpu')
        if results is not None:
            for result in results:
                if result.boxes is not None and len(result.boxes.cpu().numpy()) > 0:
                    boxes = result.boxes.cpu().numpy()
                    # x y 归一化坐标
                    normalized_xy = boxes.xywhn[:, :2]
                    classes = boxes.cls
                    name = classes[0] + 1
                    self.location_array[self.cnt] = name
                    if name == 1: # lake
                        self.lake_point = navi.current_point
                        self.wildfire_mission()
                    if name == 4: # wildfire
                        fc.set_indicator_led(255,0,0)
                        fc.set_digital_output(bi,True)
                        time.sleep(1)
                        fc.set_indicator_led(0,0,0)
                        fc.set_digital_output(bi,False)
                        self.wildfire_point = navi.current_point
                        self.wildfire_mission()
                    if name == 5: # debris
                        dx = normalized_xy[0][0]
                        dy = normalized_xy[0][1]
                        k = 0.32 # k根据高度调整, 0.27 <-> 120cm
                        self.debris_point = (navi.current_x - dy*k + 27,navi.current_y - dx*k +12)
                        logger.info(f"debris_point=={self.debris_point}")
                        # self.debris_point = navi.current_point
                        self.debris_mission()

    def wildfire_mission(self):
        fc = self.fc
        navi = self.navi
        # 如果在发现火源前有水源，或在发现火源后发现水源
        if self.lake_point is not None and self.wildfire_point is None:
            logger.info(f"found lake at {self.lake_point},but no wildfire!")
        if self.wildfire_point is not None and self.lake_point is None:
            logger.info(f"found wildfire at {self.wildfire_point},but no lake!")
        if self.lake_point is not None and self.wildfire_point is not None:
            # navi.navigation_to_waypoint(self.lake_point,wait=True)
            # 降落取水
            navi.pointing_landing(self.lake_point)
            time.sleep(3)
            # 重新起飞
            fc.set_indicator_led(0,0,255) # 亮蓝灯代表取水完成
            self.bibi()
            navi.pointing_takeoff(self.lake_point,HEIGHT)
            navi.navigation_to_waypoint(self.wildfire_point)
            navi.set_height(100)
            navi.wait_for_height()
            fc.set_digital_output(bi,True)
            time.sleep(1.5)
            fc.set_digital_output(bi,False)
            fc.set_indicator_led(0,0,0) # 熄蓝灯代表放水结束
            navi.set_height(HEIGHT)
            navi.wait_for_height()
            self.wildfire_point = None # 此处山火已熄灭
        # 如果前面没有水源，什么都不用做，继续走即可

    def debris_mission(self):
        fc = self.fc
        navi = self.navi
        navi.navigation_to_waypoint(self.debris_point)
        navi.set_height(80)
        navi.wait_for_height()
        fc.set_digital_output(ci,False)
        navi.set_height(HEIGHT)
        navi.wait_for_height()
            
if __name__ == "__main__":
    rm = RosManager()
    rm.chmod("/dev/ttyUSB0")
    rm.chmod("/dev/ttyACM1")
    rm.chmod("/dev/video1")
    rm.launch_package("ldlidar_stl_ros2", "ld06.launch.py")
    rm.launch_package("realsense2_camera", "rs_launch.py")
    rm.launch_package("cartographer_ros", "cartographer.launch.py")
    rm.run_package("tf2_ros", "static_transform_publisher", "0 0 0 0 0 0 camera_pose_frame base_link")
    fc = FC_Client()
    fc.connect()
    time.sleep(0.5)
    t265 = T265("ros")
    t265.start()
    radar = LD_Radar()
    radar.start("ros")
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
    time.sleep(1)
    fc.close()
