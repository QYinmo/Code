"""
robocup,避障无人机
"""
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
from FlightController.Solutions.Radar_find_obs import radar_resolve_obs, radar_resolve_wall
from FlightController.Solutions.Radar_SLAM import radar_resolve_rt_pose
cfg = ConfigManager(section="mission")
timer = HighPrecisionFPS()
BASE_CALI_POINT = cfg.get_array(
    "point-base", default=np.array([81.28021476, 418.3601995]))

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
MISSION_POINTS: np.ndarray = np.array(
    [ [-30, -70], [-130, -280], [80, -200]])

NEXT_MISSION_POINTS: np.ndarray = np.array([

])
SPEED = 15
HEIGHT = 80


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.rs: T265 = kwargs["rs"]
        self.navi: Navigation = kwargs["navi"]
        self.model = YOLO(
            r"/home/n1/workplace/Drone_maindev/python_sdk/robocup_best.pt")
        self.mode = 0
        self.use_serial = True
        self.started = False
        self.conf_thres = 0.9
        self.identify_status = False
        self.area_count = 0
        self.not_see = True
        self.area_timeout = 0
        self.area_res = -1
        self.break_navi = False
        self.challenge = False  # 是否做坦克任务
        self.bias = 20  # 坦克投物需要的预判量
        self.package_num = 0  # 投到第几个包了0~2
        self.package_pos = {0: [0, 0], 1: [0, 0], 2: [0, 0]}  # 每个包需要的偏置，还没写还没测
        self.ob_count = 0  # 确认有
        self.res_count = 0  # 确认对
        self.timeout = 0  # 超时
        self.last_res = -1
        self.res = -1
        self.already = np.array([])  # 记录已经查询过的点
        self.scope = 80  # 在多少范围内认为是同一个点
        self.allow_repeat = True  # 是否不需要查重
        self.use_pid = True  # 是否pid到位置
        self.next_point_event = threading.Event()
        self.obstacle_gen = ObstacleGenerator(
            x_range1=-400, x_range2=400, y_range1=-700, y_range2=200)
        self.gpio_com = [170, 0, 0, 0, 255]  # laser,buzzer,electric123
        self.x_pid = PID(Kp=0.2, Ki=0.001, Kd=0.05, setpoint=0)
        self.y_pid = PID(Kp=0.2, Ki=0.001, Kd=0.05, setpoint=0)
        self.x_pid.output_limits = (-50, 50)  # 限制输出范围
        self.y_pid.output_limits = (-50, 50)
        self.x_pid.sample_time = 0.08
        self.y_pid.sample_time = 0.08
        if self.use_serial:
            self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)
        if self.fc.last_command_done:
            pass

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def electromagnet(self, num):
        self.gpio_com[num+1] = 0
        logger.info(f"package {num} off")
        time.sleep(0.5)
        self.package_num += 1

    def init_obs(self):
        """初始化障碍物"""
        self.obstacle_gen.clear_obstacles()
        point = []
        # r = 5
       #  point.append(self.radar.map.find_nearest_with_ext_point_opt(
       #         45, 135, 3)[0].to_xy() / 10)
        # for i in range(3):
        #    point.append(self.radar.map.find_nearest_with_stable_ext_point(
        #         45, 110, 3, 4300)[i].to_xy() / 10)
        for i in range(10):
            img = self.radar.map.output_cloud(0.1, 1200)
        debug_imshow(img, "Radar")
        point = radar_resolve_obs(img, True, True)


        # degree = [0, 0, 0]
        # for i in range(3):
        #     degree[i] = int(np.arctan2(point[i][0], point[i][1]))
        for i in range(len(point)):
            x = int(point[i][0]+self.navi.current_x)
            y = int(point[i][1]+self.navi.current_y)
        # point = np.array(point)
            logger.debug(f"loc: {x}, {y}")
            self.add_obs(x, y)
        self.obs()

    def add_obs(self, x1, y1):
        self.obstacle_gen.add_obstacle(
            'circle', cx=x1, cy=y1, radius=50, filled=True)

    def obs(self):
        self.obstacle_gen.add_obstacle('wall', thickness=1)
        self.obstacles = self.obstacle_gen.get_obstacles()

    def Astar_plan(self, start, goal):
        """A*规划路径"""
        start = np.array(start)
        goal = np.array(goal)
        start_tuple = tuple(start) if isinstance(start, np.ndarray) else start
        goal_tuple = tuple(goal) if isinstance(goal, np.ndarray) else goal
        planner = AStar(start_tuple, goal_tuple, "euclidean")
        planner.update_obs(self.obstacles)
        path, visit = planner.searching()
        path = path[::-1]
        path = np.array(path)
        logger.info(f"[MISSION] A* Path planning result: {path}")
        if path.size == 0:
            logger.error("[MISSION] A* Path planning failed")
            return None
        return path

    def back_to_base(self):
        path_back = self.Astar_plan(self.navi.current_point, BASE_POINT)
        self.navi.navigation_follow_trajectory(path_back, wait=True)
        self.navi.pointing_landing(BASE_POINT)
        time.sleep(1)

    def target(self):
        logger.info("找到东西，开始瞄准")
        time_out = 0
        K = 6  # 需要实际测量 K=此高度下某像素长度/对应的实际长度单位cm
        state = 0
        navi = self.navi
        fc = self.fc
        fc.set_rgb_led(255, 255, 0)
        while True:
            ret, img = self.cam.read()
            if not ret:
                logger.warning("没有图像")
                continue
            point = navi.current_point
            f, dx, dy, data = self.identify_area(img)
            if f:
                time_out = 0
                dx = dx + \
                    self.package_pos[self.package_num][0]
                dy = dy + \
                    self.package_pos[self.package_num][1]
                if state == 0 and abs(dx) < 15 and abs(dy) < 15:  # 到位
                    if self.area_res == data:
                        logger.info(f"[MISSION] Reached point at {point}")
                        navi.set_height(40)
                        state = 1
                    else:
                        self.area_res = data
                        logger.info("确认一下")

                if state == 1 and abs(navi.current_height - navi.height_pid.setpoint) < 8:  # 到高
                    logger.info(f"[MISSION] Reached height at {point}")
                    fc.set_rgb_led(255, 0, 0)
                    state = 2
                if state == 2:  # 做任务
                    logger.info("做任务了")
                    self.do_task()
                    if not self.allow_repeat:
                        self.already = np.append(
                            self.already, navi.current_point)
                    return
                to_point = (point[0] + dx / K, point[1] - dy / K)
                navi.direct_set_waypoint(to_point)
            else:
                time_out += 1
                if time_out > 2:
                    logger.info("看错了")
                    return
            time.sleep(0.08)

    def tank_mession(self):  # 还没改
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
            point = navi.current_point
            f, dx, dy, res = self.identify_area(img)

            if f:
                # 检测当前飞机飞行方向
                the = 1  # 弧度
                dx = dx + \
                    self.package_pos[self.package_num][0]+self.bias*np.sin(the)
                dy = dy + \
                    self.package_pos[self.package_num][1]+self.bias*np.cos(the)
                if state == 0 and abs(dx) < 15 and abs(dy) < 15:
                    logger.info(f"[MISSION] Reached fire at {point}")
                    navi.set_height(80)
                    state = 1
                if state == 1 and abs(navi.current_height - navi.height_pid.setpoint) < 8:
                    logger.info(f"[MISSION] Reached height at {point}")
                    time_found = time.perf_counter()
                    fc.set_rgb_led(255, 0, 0)
                    state = 2
                if state == 2 and time.perf_counter() - time_found > 3:
                    logger.info(f"[MISSION] Released pack at {point}")
                    self.electromagnet(self.package_num)
                    fc.set_rgb_led(0, 0, 0)
                    navi.set_height(self.cruise_height)
                    navi.wait_for_height()
                    self.send_running = True
                    return
                to_point = (point[0] - dy / K, point[1] - dx / K)
                navi.direct_set_waypoint(to_point)
            time.sleep(0.08)

    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()

    def identify_area(self, img):
        try:
            debug_imshow(img, "Origin")
            results = self.model(source=img, show=False, conf=self.conf_thres, save=False,
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
                        debug_imshow(annotated_frame, "Result")
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
                            return True, (res[0]-0.5)*img.shape[1], (res[1]-0.5)*img.shape[0], res[2]
                    else:
                        print("没东西")
                        return False, 0, 0, -1
        except Exception as e:
            logger.error(f"识别失败: {e}")
            logger.warning("死了")
            return False, 0, 0, -1

    def do_task(self):
        fc = self.fc
        navi = self.navi
        if self.area_res == 0:
            logger.info(f"发现帐篷")
        if self.area_res == 1:
            logger.info(f"发现地堡")
            if not self.challenge:
                self.electromagnet(self.package_num)
                fc.set_rgb_led(0, 0, 0)
                navi.set_height(self.cruise_height)
                navi.wait_for_height()
                logger.info(
                    "[MISSION] Pack dropped, continue trajectory")
        if self.area_res == 4:
            logger.info(f"发现桥")
            self.electromagnet(self.package_num)
            fc.set_rgb_led(0, 0, 0)
            navi.set_height(self.cruise_height)
            navi.wait_for_height()
            logger.info(
                "[MISSION] Pack dropped, continue trajectory")
        if self.area_res == 2:
            logger.info(f"发现坦克")
            if self.challenge:
                self.tank_mession()
                logger.info(
                    "[MISSION] Pack dropped, continue trajectory")
        if self.area_res == 3:
            logger.info(f"发现车")
            self.electromagnet(self.package_num)
            fc.set_rgb_led(0, 0, 0)
            navi.set_height(self.cruise_height)
            navi.wait_for_height()
            logger.info(
                "[MISSION] Pack dropped, continue trajectory")

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
                f, _, _, res = self.identify_area(img)
                if f:
                    self.ob_count += 1
                    self.timeout = 0
                else:
                    self.ob_count = 0
                    self.timeout += 1
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
                        logger.info("有东西了")
                        time.sleep(0.1)
                        self.area_res = int(res)
                        self.target()
                        self.next_point_event.set()
                if self.timeout:
                    logger.warning("此处没识别到任何")
                    self.next_point_event.set()
            time.sleep(0.06)
            print(f"瞬时FPS: {timer.fps():.1f}")
            timer.reset()

    def path_3d(self, path=None):

        height_column = np.ones((len(path), 1)) * HEIGHT
        path_3d = np.hstack((path, height_column))
        logger.info(f"path{path_3d}")
        return path_3d

    def next_mission(self):
        path_go = self.Astar_plan(
            self.navi.current_point, NEXT_MISSION_POINTS[0])
        self.navi.navigation_follow_trajectory(path_go, wait=True)

    def run(self):
        fc = self.fc
        cam = self.cam
        navi = self.navi

        self.navigation_speed = SPEED
        self.cruise_height = HEIGHT
        self.vertical_speed = 15
        vision_debug(True)
        self.inital_yaw = self.fc.state.yaw.value
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()
        navi.switch_navigation_mode("fusion-ros")
        navi.set_basepoint(BASE_POINT)
        navi.set_rs_speed_report(True, 2)
        fc.set_action_log(False)
       # set_cam_autowb(cam, True)
        set_manual_exporsure(cam, -7)
        fc.set_indicator_led(0, 255, 0)
        self.start_camera_task()

        time.sleep(0.05)
        # self.serial_terminal.listen_start(self.rxbuffer)
        self.serial_gpio.send_start(self.gpio_com)
        time.sleep(0.1)
        self.gpio_com[1] = 1
        self.gpio_com[2] = 1
        self.gpio_com[3] = 1



        time.sleep(5)
    
        self.started = True
        for _ in range(3):
            time.sleep(0.25)
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(0.25)
            fc.set_indicator_led(0, 0, 0)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")

        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        navi.wait_for_height()
        self.init_obs()
        time.sleep(0.1)
        logger.info("初始化，找到障碍物")
        points = MISSION_POINTS[0]
        for  points in MISSION_POINTS:
            logger.info(f"[MISSION] Go to point {points}")
            path = self.Astar_plan([int(self.navi.current_x//5*5),int(self.navi.current_y//5*5)], points)
            logger.info("jiesuanwancheng")
            if path is not None and len(path):
                    path = self.path_3d(path)
                    
            else:
                    logger.warning(
                        f"[MISSION] Path planning failed for point {points}. Skipping navigation.")
                    break
            logger.info("kaishidaohang")
            navi.navigation_follow_trajectory(path, wait=True, dt=0.1)
            self.next_point_event.clear()
            self.identify_status = True
            logger.info("到点，识别")
            self.next_point_event.wait()
            self.next_point_event.clear()
        next_points = BASE_POINT
        path = self.Astar_plan([int(self.navi.current_x//5*5),int(self.navi.current_y//5*5)], [0,0])
        if path is not None and len(path):
                    path = self.path_3d(path)
        else:
                    logger.warning(
                        f"[MISSION] Path planning failed for home. ")
                    
        navi.navigation_follow_trajectory(path, wait=True, dt=0.1)
        navi.pointing_landing([0,0])
        time.sleep(0.1)


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
