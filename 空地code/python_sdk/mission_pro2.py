"""
测绘无人机
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
from trash.usr_serial import  Serial_gpio, Serial_station
from FlightController.Components.RosManager import RosManager 
from ultralytics import YOLO
import onnxruntime as ort



BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
SPEED = 25
HEIGHT = 150
X_SIZE=np.array([90, 0,0])
Y_SIZE=np.array([0, -90,0])
P=lambda x, y: np.array([-20, 20,HEIGHT]) + X_SIZE * x + Y_SIZE * y
MISSION_POINTS =  np.array([
P(1,1),P(1,2),P(1,3),P(1,4),P(2,4),P(3,4),P(3,3),P(2,3),P(2,2),P(3,2),P(3,1),P(2,1),
])

classes = {
    0: 'vocano',
    1: 'mountain',
    2: 'lake',
    3: 'river',
    4: 'farmland',
    5: 'village',
    6: 'mudslide'
}
model_path = r"./best.onnx"
conf_thres = 0.7


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.rs: T265 = kwargs["rs"]
        self.navi: Navigation = kwargs["navi"]
        self.mode = 0
        self.area_timeout=0
        self.area_count = 0
        self.area_res = -1
        self.identify_status = False 
        self.use_serial = True
        self.gpio_com = [170, 0,0,0, 255] #laser,buzzer,citie
        self.img = self.cam.read()
        self.next_point_event = threading.Event()
        if self.use_serial:
            self.serial_terminal = Serial_station(
                device="cp2102", baudrate=115200, rx_length=14)
            self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)
        if self.fc.last_command_done:
            pass
        self.location = 0 # 无人机位置计数
        self.started = False
        self.terminal_com =[170,10,10,10,10,10,10,10,10,10,10,10,10,255]
        # 水源位置
        self.lake_point = None
        # 火源位置
        self.wildfire_point = None
        # 泥石流位置
        self.mudslide_point = None
        self.model = YOLO(model_path, task='detect')
    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()


    def identify_area(self, img, model):
        try:
            results = model(source=img, show=False, conf=conf_thres, save=False,
                            verbose=False, stream=True, device='cpu')
            processed_results = []
            final_results = []  # 用于存储最终结果
            if results is not None:
                for result in results:
                    if result.boxes is not None and len(result.boxes.cpu().numpy()) > 0:
                        boxes = result.boxes.cpu().numpy()

                        normalized_xy = boxes.xywhn[:, :2]
                        classes = boxes.cls
                        confs = boxes.conf
                        annotated_frame = result.plot()  # 自动绘制boxes/labels
                        cv2.imshow('YOLOv8 Real-Time Detection', annotated_frame)
                        for i in range(len(classes)):
                            processed_results.append([
                                normalized_xy[i][0],       # x坐标
                                normalized_xy[i][1],       # y坐标
                                int(classes[i]),            # 类别
                                float(confs[i])            # 置信度
                            ])


                        processed_results = np.array(processed_results)

                        cls_results = processed_results
                        max_conf_idx = np.argmax(cls_results[:, 3])
                        best_result = cls_results[max_conf_idx]
                        final_results.append(best_result)
                        final_results = [list(row) for row in final_results]
                        for res in final_results:
                            print(
                                f"类别: {int(res[2])}, 坐标: ({res[0]:.5f}, {res[1]:.5f}), 置信度: {res[3]:.5f}")
                            return res[2]
                    else:
                        print("没东西")
                        return None
        except:
            logger.warning("死了")
            return None


    def camera_task(self):
        res_pre = -1
        while True:
            ret, img = self.cam.read()
            # img = img[50:200, 100:300]
            if self.identify_status:
                res = self.identify_area(img,self.model)
                if res == None:
                    continue
                else:
                    if res == res_pre:
                        self.area_count += 1
                        self.area_timeout = 0
                    else:
                        self.area_timeout += 1
                        res_pre = res
                    if self.area_count > 1:
                        self.area_count = 0
                        self.area_timeout = 0
                        self.identify_status = False
                        if self.started:
                            self.area_res = int(res)
                            self.terminal_com[self.location+1] = self.area_res
                            if self.area_res == 2:
                                self.lake_point = MISSION_POINTS[self.location]
                                logger.info(f"发现水源{self.lake_point}")
                                self.wildfire_mission()
                            if self.area_res == 0:
                                self.wildfire_point = MISSION_POINTS[self.location]
                                logger.info(f"发现火源{self.wildfire_point}")
                                self.wildfire_mission()  
                            if self.area_res == 6:
                                self.mudslide_point = MISSION_POINTS[self.location]
                                logger.info(f"发现泥石流{self.mudslide_point}")
                                self.mudslide_mission()  
                            print("这是", classes[int(res)])
                            self.next_point_event.set()
                    elif self.area_timeout > 2:
                        self.area_timeout = 0
                        self.area_count = 0
                        self.identify_status = False
                        if self.started:
                            self.area_res = -1
                            logger.warning("太难了，我不会")
                            self.next_point_event.set()
                            
    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")


    def electromagnet(self):
        self.gpio_com[3] = 0
        logger.info("[MISSION] Electromagnet mission")
        




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
        self.gpio_com[2]=1
        time.sleep(0.5)
        self.gpio_com[2]=0
        time.sleep(0.5)
        self.gpio_com[2]=1
        time.sleep(0.5)
        self.gpio_com[2]=0
        time.sleep(0.5)
        #蜂鸣

    def run(self):
        fc = self.fc
        radar = self.radar
        navi = self.navi
        self.model = YOLO(model_path, task='detect')
        ############### 参数 #################
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
        ################  校准 ################
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(False)
        fc.set_indicator_led(0, 255, 0)
        time.sleep(0.25)       
        threading.Thread(target=self.carto_check, daemon=True).start()
        self.started = True
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")
        self.serial_terminal.start_transmit(
            self.terminal_com)
        self.serial_gpio.send_start(self.gpio_com)
        self.start_camera_task()
        self.gpio_com[3]=1
        time.sleep(0.8)
        ################ 初始化完成 ################
        for _ in range(3):
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(1)
            fc.set_indicator_led(0, 0, 0)  # 起飞前警告
            time.sleep(1)
        self.navi.set_basepoint(BASE_POINT)
        self.navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        for self.location in range(len(MISSION_POINTS)):
            navi.navigation_to_waypoint(MISSION_POINTS[self.location ])
            self.navi.wait_for_waypoint()
            logger.info(f"[MISSION] Go to target point {MISSION_POINTS[self.location ]}")
            self.next_point_event.clear()
            self.identify_status = True
            self.next_point_event.wait()
            self.next_point_event.clear()
            fc.set_indicator_led(0,0,255) # 蓝灯提示
            time.sleep(0.5)
            fc.set_indicator_led(0,0,0)
        navi.pointing_landing(LANDING_POINT)

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
            time.sleep(1)
            # 重新起飞
            fc.set_indicator_led(0,0,255) # 亮蓝灯代表取水完成
            self.Buzzer()
            navi.pointing_takeoff(self.lake_point,HEIGHT)
            navi.navigation_to_waypoint(self.wildfire_point)
            navi.set_height(100)
            navi.wait_for_height()
            self.gpio_com[2] = 1 
            time.sleep(0.5)
            self.gpio_com[2] = 0
            fc.set_indicator_led(0,0,0) # 熄蓝灯代表放水结束
            navi.set_height(HEIGHT)
            navi.wait_for_height()
            self.wildfire_point = None 
            logger.info("wildfire mission finished")


    def mudslide_mission(self):
        fc = self.fc
        navi = self.navi
        navi.set_height(80)
        navi.wait_for_height()
        self.laser(1)
        time.sleep(0.5)
        self.laser(0)
        time.sleep(0.5)
        self.electromagnet()
        time.sleep(0.5)
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
    cam, i = open_camera()
    t265.start()
    radar = LD_Radar()
    radar.start("ros")
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
        cam=cam,
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
