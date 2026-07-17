"""
植保无人机
"""

import threading
import time
from typing import List
import os
import cv2
import numpy as np
from FlightController.Components.RosManager import RosManager
from FlightController import FC_Client, FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Solutions.Navigation import Navigation
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger
from trash.usr_serial import  Serial_gpio
BASE_POINT:np.array = np.array([0, 0])
LANDING_POINT:np.array = np.array([0, 0])

SPEED = 35
HEIGHT = 120
X_SIZE=np.array([50, 0,0])
Y_SIZE=np.array([0, -50,0])
P=lambda x, y: np.array([-35, -10,HEIGHT]) + X_SIZE * x + Y_SIZE * y
target_points = np.array([
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
])

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
        self.gpio_com = [170, 0, 255] #laser
        self.green_lower = np.array([30, 10, 50])
        self.green_upper = np.array([99, 210, 255])
        self.green_ratio_ref = 0.4
        self.green_count = 0
        self.green_timeout = 0
        self.identify_status = False
        self.img = self.cam.read()
        self.next_point_event = threading.Event()
        if self.use_serial:
            self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)
        if self.fc.last_command_done:
            pass

    def carto_check(self):
        while True:
            if self.mode ==0:
                if (self.navi.mapper._trans_node.transform_established is False):
                    self.fc.set_indicator_led(255, 255, 0)
                    logger.info("carto error")
                else:
                    self.fc.set_indicator_led(255, 255, 255)
                    logger.info("carto ok")
            if self.started:
                break
            time.sleep(0.3)
            #carto检测

    def start_camera_task(self):
        threading.Thread(target=self.take_photo, daemon=True).start()

    def identify_green(self, img):
     try:
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv_image, self.green_lower, self.green_upper)
        green_result=cv2.bitwise_and(img, img, mask=green_mask)
        cv2.imwrite("green_result.jpg", green_result)
        cv2.imwrite("img.jpg", img)
        print("图像已保存")
        total_pixels = green_mask.size  # 总像素数
        green_pixels = cv2.countNonZero(green_mask)  # 绿色区域的像素数
        green_ratio = green_pixels / total_pixels  # 绿色区域的比例
        print("绿色区域的比例:",green_ratio,"总像素数",total_pixels)
        if green_ratio > self.green_ratio_ref:
            print("绿色区域占比超过ref")
            return 1
        else:
            print("绿色区域占比不超过ref")
            return 0
     except:
        logger.warning("死了")
        return None

    def take_photo(self):
      t = 0
      image_counter = 0
      while True:
        t += 1
        ret, img = cam.read()
        cv2.imshow('test', img)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}_{image_counter}.png"
        if t == 30:
            t = 0
            image_counter += 1
            cv2.imwrite(filename, img)
            print(f"已保存图像：{filename}")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cam.release()
            cv2.destroyAllWindows()
            break
    
    def laser(self, status):
        self.gpio_com[1] = status
        if status:
            logger.info("[MISSION] Laser ON")
        else:
            logger.info("[MISSION] Laser OFF")
    

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")


    def run(self):
        rm = RosManager()
        rm.chmod("/dev/ttyUSB0")
        rm.chmod("/dev/ttyACM1")
        rm.chmod("/dev/video1")
        rm.launch_package("ldlidar_stl_ros2", "ld06.launch.py")
        rm.launch_package("realsense2_camera", "rs_launch.py")
        rm.launch_package("cartographer_ros", "cartographer.launch.py")
        rm.run_package("tf2_ros", "static_transform_publisher", "0 0 0 0 0 0 camera_pose_frame base_link")
        fc = self.fc
        radar = self.radar
        cam = self.cam
        self.navigation_speed = SPEED  # 导航速度
        self.cruise_height = HEIGHT  # 巡航高度
        fc.set_flight_mode(fc.PROGRAM_MODE)
        self.navi.set_navigation_speed(self.navigation_speed)
        self.navi.set_vertical_speed(35)
        self.navi.start()  # 启动导航线程
        self.navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started")

        self.serial_gpio.send_start(self.gpio_com)
        threading.Thread(target=self.carto_check, daemon=True).start()
        fc.set_action_log(False)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 450.0)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 250.0)
        self.start_camera_task()
        #cam.set(cv2.CAP_PROP_AUTO_WB, True)
        logger.info("[MISSION] Serial received . Waiting for takeoff signal")
        fc.set_indicator_led(255, 255, 255)  # 等待起飞命令
        self.started = True
        logger.info("[MISSION] Mission Started")
        for _ in range(3):
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(1)
            fc.set_indicator_led(0, 0, 0)  # 起飞前警告
            time.sleep(1)
        fc.set_action_log(True)
        self.navi.set_basepoint(BASE_POINT)
        navi.set_rs_speed_report(True, 2)
        self.navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        for target_point in target_points:
            self.navi.navigation_to_waypoint(target_point)
            self.navi.wait_for_waypoint()
            logger.info(f"[MISSION] Go to target point {target_point}")
            time.sleep(2)
        logger.info("[MISSION] Go to landing point")
        self.navi.navigation_to_waypoint(LANDING_POINT)
        self.navi.wait_for_waypoint()
        self.navi.set_navigation_speed(20)
        self.navi.pointing_landing(LANDING_POINT)
        logger.info("[MISSION] Misson Finished")

if __name__ == "__main__":
     fc = FC_Client()
     fc.connect()
     fc.wait_for_connection()
     t265 = T265("ros")
     t265.start()
     radar = LD_Radar()
     radar.start("ros")
     cam, i = open_camera()
     logger.info(f"Camera {i} opened")
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