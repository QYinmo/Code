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
from mission_2025_draft import xy_to_ab, id_to_xy
timer = HighPrecisionFPS()
cfg = ConfigManager(section="mission")

BASE_CALI_POINT = cfg.get_array(
    "point-base", default=np.array([81.28021476, 418.3601995]))

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
MISSION_POINTS: np.ndarray = np.array([])
def P(x, y): return np.array([x * 50, y * 50, HEIGHT])


X_SIZE = np.array([50, 0, 0])
Y_SIZE = np.array([0, 50, 0])
SPEED = 20
HEIGHT = 105


ANIMAL_CLSASS = {
    0: "象",
    1: "虎",
    2: "狼",
    3: "猴",
    4: "孔雀",
}


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
        self.res = -1
        self.already = set()  # 记录已经查询过的点
        self.scope = 10  # 在多少范围内认为是同一个点
        self.allow_repeat = True  # 是否不需要查重
        self.conf_thres = 0.9
        self.area_count = 0
        self.gpio_com = [170, 0, 255]  # laser
        self.terminal_com = [170, 0, 0, 0, 0, 0, 0, 0, 255]  # 当前位置a,b，象虎狼猴孔雀分别对应位置的数量
        self.terminal_rxbuffer = [0]
        if self.use_serial:
            self.serial_terminal = Serial_station(
                device="cp2102", baudrate=115200)
            self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)
        if self.fc.last_command_done:
            pass

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")
    # ------gpio-----

    def laser(self, status):
        self.gpio_com[1] = status
        if status:
            logger.info("[MISSION] Laser ON")
        else:
            logger.info("[MISSION] Laser OFF")

    # ------视觉------

    def id_function(self, img, model):
        try:
            debug_imshow(img, "Origin")
            results = model(source=img, show=False, conf=self.conf_thres, save=False,
                            verbose=False, stream=True, device='cpu')
            all_detected_objects = []
            if results is not None:
                for result in results:
                    if result.boxes is not None and len(result.boxes.cpu().numpy()) > 0:
                        boxes = result.boxes.cpu().numpy()
                        xy = boxes.xywh[:, :2]
                        classes = boxes.cls
                        confs = boxes.conf
                        annotated_frame = result.plot()  # 自动绘制boxes/labels
                        debug_imshow(annotated_frame, "Result")
                        for i in range(len(classes)):
                            all_detected_objects.append([
                                xy[i][0]-img.shape[1],       # dx
                                xy[i][1]-img.shape[0],       # dy
                                int(classes[i]),            # 类别
                                float(confs[i])            # 置信度
                            ])
                    else:
                        pass
            if not all_detected_objects:
                logger.info("没有检测到任何目标")
                return False, [], {}  # 返回False，空列表，空字典
            class_counts = {}
            for obj in all_detected_objects:
                class_id = obj[2]  # 类别ID
                class_counts[class_id] = class_counts.get(class_id, 0) + 1
            for obj in all_detected_objects:
                logger.info(
                    f"类别: {ANIMAL_CLSASS[int(obj[2])]}, 坐标: ({obj[0]:.5f}, {obj[1]:.5f}), 置信度: {obj[3]:.5f}")

            logger.info("\n--- 类别统计 ---")
            for cls_id, count in class_counts.items():
                logger.info(f"类别 {ANIMAL_CLSASS[cls_id]}: {count} 个")
            return True, all_detected_objects, class_counts

        except Exception as e:
            logger.error(f"识别失败: {e}")
            logger.warning("死了")
            return False, [], {}

    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()

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
                f, _, _ = self.id_function(img, self.model)
                if f:
                    self.ob_count += 1
                else:
                    self.ob_count = 0
                
                if self.ob_count > 1:  # 此处改确认存在次数
                    self.ob_count = 0
                    self.timeout = 0
                    self.identify_status = False
                    if self.started:
                        logger.info("停下导航")
                        self.navi.navigation_stop_here()
                        time.sleep(0.05)
                        _, pos, statistic = self.id_function(img, self.model)
                        self.record(pos, statistic)
                        self.navi.navigation_follow_trajectory(
                            self.navi.traj_list_before_stop, wait=False)
                if self.timeout > 3:
                    self.ob_count = 0
                    self.timeout = 0
                    self.identify_status = False
                    logger.info("此处没动物")
                    self.navi.navigation_follow_trajectory(
                        self.navi.traj_list_before_stop, wait=False)

            time.sleep(0.06)
            print(f"瞬时FPS: {timer.fps():.1f}")
            timer.reset()
# ------处理与任务-------

    def record(self, positon, statistic):  # 记录数据
        for i, obj_info in enumerate(positon):
            dx = obj_info[0]
            dy = obj_info[1]
            class_id = obj_info[2]
            logger.info(f"目标 {i+1}:")
            logger.info(f"  类别 ID: {class_id}")
            logger.info(f"  dx,dy: ({dx:.5f}, {dx:.5f})")
            self.target(dx, dy)
        logger.info("动物照完了")
        logger.info("\n--- 类别统计 ---")
        if statistic:  # 检查字典是否为空
            for cls_id, count in statistic.items():
                logger.info(f"类别 {cls_id}: 共 {count} 个")
                self.terminal_com[cls_id+3] = count

        else:
            logger.info("没有可用的类别统计数据。")

    def target(self, dx, dy):  # 照动物
        K = 4.61  # 实地考察计算
        navi = self.navi
        point = navi.current_point
        to_point = (point[0] - dy / K, point[1] - dx / K)
        navi.direct_set_waypoint(to_point)
        time.sleep(1)
        navi.direct_set_waypoint(point)
        logger.info("下一个动物")
# -------run------

    def get_route(self):  # 根据串口信号转化为任务轨迹
        list_of_processed_arrays = []
        DT = 0.2
        test=[
[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [6, 1], [5, 1], [4, 1], [3, 1], [2, 1], [1, 1], [1, 2], [2, 2], [3, 2], [4, 2], [5, 2], [6, 2], [6, 3], [5, 3], [4, 3], [3, 3], [2, 3], [1, 3], [1, 4], [2, 4], [3, 4], [4, 4], [5, 4], [6, 4], [6, 5], [6, 4], [5, 4], [4, 4], [4, 5], [3, 5], [2, 5], [1, 5], [1, 6], [2, 6], [3, 6], [4, 6], [4, 5], [4, 4], [5, 4], [6, 4], [6, 5], [6, 6], [6, 7], [6, 8], [5, 8], [4, 8], [4, 7], [3, 7], [2, 7], [1, 7], [1, 8], [2, 8], [3, 8], [2, 8], [1, 8], [0, 8], [0, 7], [0, 6], [0, 5], [0, 4], [0, 3], [0, 2], [0, 1],[0,0]
]
        mission_path: List[Tuple[float, float, float]] = []
        mission_path.append((0, 0, HEIGHT))
        for element in test:
            point = element
            single_np_array = P(*point)
            list_of_processed_arrays.append(single_np_array)

        if list_of_processed_arrays:  # 确保列表不为空，避免对空列表进行堆叠
            final_stacked_array = np.stack(list_of_processed_arrays)
            for i in range(len(final_stacked_array) - 1):
                    last_p = final_stacked_array[i]
                    next_p = final_stacked_array[(i + 1)]
                    length = np.linalg.norm(next_p - last_p)
                    traj_g = TrajectoryGenerator(last_p, next_p, length / SPEED)
                    traj_g.solve()
                    t = 0.0       
                    while t < length / SPEED:
                         mission_path.append(traj_g.calc_position_xyz(t))
                         t += DT
        else:
            final_stacked_array = None
            logger.error("飞行路径不存在")
        return mission_path

    def run(self):
        fc = self.fc
        cam = self.cam
        navi = self.navi

        self.navigation_speed = SPEED
        self.cruise_height = HEIGHT
        self.vertical_speed = 15

        self.inital_yaw = self.fc.state.yaw.value
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()
        navi.switch_navigation_mode("fusion-ros")
        navi.set_basepoint(BASE_POINT)
        navi.set_rs_speed_report(True, 4)
        fc.set_action_log(False)

        fc.set_indicator_led(0, 255, 0)

        time.sleep(0.05)
        self.serial_terminal.start_transmit(
            self.terminal_com, self.terminal_rxbuffer)
        self.serial_gpio.send_start(self.gpio_com)
        time.sleep(0.1)
        

        
        self.gpio_com[1] = 1
        self.started = True
        for _ in range(3):
            time.sleep(0.25)
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(0.25)
            fc.set_indicator_led(0, 0, 0)
        fc.set_action_log(True)
        mission_path=self.get_route()
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        time.sleep(0.3)
        navi.navigation_follow_trajectory(mission_path, wait=True)
        navi.pointing_landing(BASE_POINT)
        

        #################


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
