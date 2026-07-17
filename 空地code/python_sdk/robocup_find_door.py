"""
robocup,避障无人机
"""
import random
import struct
import threading
import time
from typing import List, Tuple
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
from get_cam_index import get_sy_1080p_camera, get_usb_2_0_camera
cfg = ConfigManager(section="mission")
timer = HighPrecisionFPS()

H_PRIME_FILE = 'H_prime_LS_matrix_new.npy'
H_PRIME_MATRIX = None  # 初始化为 None

try:
    if not os.path.exists(H_PRIME_FILE):
        # 如果文件不存在，抛出错误，但我们会捕获并礼貌地提示用户
        raise FileNotFoundError(f"错误：未找到透视矩阵文件 '{H_PRIME_FILE}'。请先运行第一个代码文件。")

    # 加载高精度二进制矩阵
    H_PRIME_MATRIX = np.load(H_PRIME_FILE)

except FileNotFoundError as e:
    # 打印错误信息
    logger.info(e)

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
MISSION_POINTS: np.ndarray = np.array(
    [ [360, -200], [-140, -560]])  # 顺时针1234

NEXT_MISSION_PATH: np.ndarray = np.array(
    [[0,0], [-180, 0], [-180, 0]])
NEXT_MISSION_POINT: np.ndarray = np.array([100, -800])
SPEED = 20
HEIGHT = 85


def transform_dxdy_to_fxfy_homography(dx: float, dy: float) -> Tuple[float, float]:
    """
    使用 Homography 将图像坐标 (dx, dy) 转换为现实坐标 (f_x, f_y)。
    """

    # 检查矩阵是否加载成功
    if H_PRIME_MATRIX is None:
        logger.warning("矩阵加载失败")
        return 0, 0

    # 输入点必须是 (1, 1, 2) 的 float64 数组，这是 cv2.perspectiveTransform 的要求
    image_point = np.array([[[dx, dy]]], dtype=np.float64)

    # 执行透视变换: H' 将 (dx, dy) 转换为 (f_y, f_x)
    world_point_transformed = cv2.perspectiveTransform(
        image_point, H_PRIME_MATRIX)

    fy = world_point_transformed[0, 0, 0]*100.0
    fx = world_point_transformed[0, 0, 1]*100.0-27  # -27是摄像头和飞机的相对位置

    return fx, fy


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.cam1: cv2.VideoCapture = kwargs["cam1"]
        self.cam2: cv2.VideoCapture = kwargs["cam2"]
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
        self.find_hospital = False  # 找红十字标志位
        self.found_hospital = False  # 找到红十字
        self.find_hospital_mission_finish = False  # 之后是否不再找红十字
        self.ori_hospital_pos = [0, 0, 0, 0]  # 红十字的近似绝对位置[当前x,y,视野dx,dy]
        self.found_coordinates = []
        self.hospital_pos = [0, 0]
        self.id_hospital = False  # 是否开始瞄准红十字
        self.package_num = 0  # 投到第几个包了0~2    0号包是红十字，右侧；1号包是左侧；2号包是中间
        self.package_pos = {0: [5, 24], 1: [
            7, -12], 2: [15, 8]}  # 每个包需要的偏置，还没写还没测
        self.ob_count = 0  # 确认有
        self.res_count = 0  # 确认对
        self.timeout = 0  # 超时
        self.last_res = -1
        self.res = -1
        self.already = np.array([])  # 记录已经查询过的点
        self.scope = 80  # 在多少范围内认为是同一个点
        self.allow_repeat = True  # 是否不需要查重
        self.use_pid = True  # 是否pid到位置
        self.closest_index=-1
        self.next_point_event = threading.Event()
        self.obstacle_gen = ObstacleGenerator(
            x_range1=550, x_range2=-500, y_range1=-1000, y_range2=100)
        self.gpio_com = [170, 0, 0, 0, 255]  # electric123
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

    def back_to_base(self):
        path_back = self.Astar_plan(self.navi.current_point, BASE_POINT)
        self.navi.navigation_follow_trajectory(path_back, wait=True)
        self.navi.pointing_landing(BASE_POINT)
        time.sleep(1)

    def electromagnet(self, num):
        self.gpio_com[num+1] = 0
        logger.info(f"package {num} off")
        time.sleep(0.5)


####### 障碍与路线规划#########
    def init_obs(self, N=0):
            """初始化障碍物"""
            if N == 0:
                self.obstacle_gen.clear_obstacles()

            # ... (此处省略雷达点云获取和障碍物中心点检测的循环，保持不变) ...
            point = []
            trymax = 5
            # ... (雷达点云获取和障碍物中心点检测的循环，保持不变) ...

            for retry in range(trymax):
                for i in range(10):
                    img = self.radar.map.output_cloud(0.1, 2000)
                debug_imshow(img, "Radar")
                point = radar_resolve_obs(img, True, True)
                if len(point) >= 3:
                    logger.info(f"已经找到{len(point)}个障碍物")
                    break
                else:
                    logger.info(f"重试第{retry+1}次")
                    if retry+1 == trymax:
                        logger.info("寻找失败！！！")

                time.sleep(0.05)

            current_x = self.navi.current_x
            current_y = self.navi.current_y
            
            # 定义延伸距离 (10 cm)
            EXTENSION_DISTANCE = 12
            if N==0:
                    for i in range(len(point)):
                        obs_rel_x = point[i][0]
                        obs_rel_y = point[i][1]
                        obs_abs_x = int(obs_rel_x + current_x)
                        obs_abs_y = int(obs_rel_y + current_y)

                        distance = np.sqrt(obs_rel_x**2 + obs_rel_y**2)

                        if distance > 0:
                            
                            unit_vec_x = obs_rel_x / distance
                            unit_vec_y = obs_rel_y / distance
                            
                            # 沿单位向量方向延伸 10 cm
                            extended_x = obs_abs_x + unit_vec_x * EXTENSION_DISTANCE
                            extended_y = obs_abs_y + unit_vec_y * EXTENSION_DISTANCE
                        else:
                            extended_x = obs_abs_x
                            extended_y = obs_abs_y
                            
                        final_x = int(extended_x)
                        final_y = int(extended_y)
                        logger.debug(f"final_loc: {final_x}, {final_y}")
                        
                        self.add_obs(final_x, final_y)
            else:
                for i in range(len(point)):
                    x = int(point[i][0]+self.navi.current_x)
                    y = int(point[i][1]+self.navi.current_y)
                # point = np.array(point)
                    logger.debug(f"loc: {x}, {y}")
                    self.add_obs(x, y)
            self.obs()


    def add_obs(self, x1, y1):
        self.obstacle_gen.add_obstacle(
            'circle', cx=x1, cy=y1, radius=85, filled=True)

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
        logger.info(f"[MISSION] A* Path planning result: {path}")
        path = np.array(path)

        if path.size == 0:
            logger.error("[MISSION] A* Path planning failed")
            return None
        return path

######## 视觉与追踪######

    def target(self):
        logger.info("找到东西，开始瞄准")
        time_out = 0
        K = 5.5  # 需要实际测量 K=此高度下某像素长度/对应的实际长度单位cm
        state = 0
        package_h = 9
        navi = self.navi
        fc = self.fc
        fc.set_rgb_led(255, 255, 0)
        if self.found_hospital is False:
            if self.area_res in {0, 2}:
                logger.info("没找到红十字，帐篷或者坦克,直接下一个点")
                return
        else:
            if self.area_res in {0, 1, 2}:
                logger.info("帐篷,地堡或者坦克,直接下一个点")
                return
        while True:
            ret, img = self.cam1.read()
            if not ret:
                logger.warning("没有图像")
                continue
            point = navi.current_point
            if state == 0:
                f, dx, dy, data = self.identify_area(img)
                if f:
                    time_out = 0
                    if state == 0 and abs(dx) < 15 and abs(dy) < 15:  # 到位
                        if self.area_res == data:
                            logger.info(f"[MISSION] Reached point at {point}")
                            if self.area_res == 4:
                                logger.info("这是桥梁")
                            if self.area_res == 3:
                                logger.info("这是车")
                            to_point = (point[0] + self.package_pos[self.package_num][0], point[1] + self.package_pos[self.package_num][1])
                            self.navi.navigation_to_waypoint(to_point, wait=True)
                            state = 1
                            continue
                        else:
                            self.area_res = data
                            logger.info("确认一下")
                    to_point = (point[0] + dx / K, point[1] - dy / K)
                    navi.direct_set_waypoint(to_point)
                else:
                    time_out += 1
                    if time_out > 2:
                        logger.info("看错了")
                        return
            if state == 1 :
                logger.info(f"[MISSION] Reached point at {point}")
                navi.set_height(package_h)
                state = 2
            if state == 2 and abs(navi.current_height - navi.height_pid.setpoint) < 8:  # 到高
                logger.info(f"[MISSION] Reached height at {point}")
                fc.set_rgb_led(255, 0, 0)
                state = 3
                continue
            if state == 3:  # 做任务
                logger.info("做任务了")
                self.do_task(is_hos=False)
                if not self.allow_repeat:
                    self.already = np.append(
                        self.already, navi.current_point)
                return

            time.sleep(0.08)

    def target_hospital(self):
        logger.info("找到东西，开始瞄准")
        K = 5.5# 需要实际测量 K=此高度下某像素长度/对应的实际长度单位cm
        state = 0
        package_h = 9
        navi = self.navi
        fc = self.fc
        fc.set_rgb_led(255, 255, 0)
        while True:
            ret, img = self.cam1.read()
            if not ret:
                logger.warning("没有图像")
                continue
            point = navi.current_point
            if state == 0:
                f, dx, dy, _ = find_red_area(img, 100)
                if f:
                    if state == 0 and abs(dx) < 10 and abs(dy) < 10:  # 到位
                        logger.info(f"[MISSION] Reached point at {point}")
                        to_point = (point[0] + self.package_pos[0][0], point[1] + self.package_pos[0][1])
                        self.navi.navigation_to_waypoint(to_point, wait=True)
                        state = 1
                        continue
                    to_point = (point[0] + dx / K, point[1] - dy / K)
                    navi.direct_set_waypoint(to_point)
            if state == 1 :
                logger.info(f"[MISSION] Reached point at {point}")
                navi.set_height(package_h)
                state = 2
            if state == 2 and abs(navi.current_height - navi.height_pid.setpoint) < 8:  # 到高
                logger.info(f"[MISSION] Reached height at {point}")
                fc.set_rgb_led(255, 0, 0)
                state = 3
                continue
            if state == 3:  # 做任务
                logger.info("做任务了")
                self.do_task(is_hos=True)
                return

            time.sleep(0.08)

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
                            logger.info(
                                f"类别: {int(res[2])}, 坐标: ({res[0]:.5f}, {res[1]:.5f}), 置信度: {res[3]:.5f}")
                            return True, (res[0]-0.5)*img.shape[1], (res[1]-0.5)*img.shape[0], res[2]
                    else:
                        logger.info("没东西")
                        return False, 0, 0, -1
        except Exception as e:
            logger.error(f"识别失败: {e}")
            logger.warning("死了")
            return False, 0, 0, -1

    def do_task(self, is_hos=False):
        fc = self.fc
        navi = self.navi
        if is_hos:
            logger.info("医院任务，直接投")
            self.electromagnet(0)
        else:
            self.electromagnet(self.package_num)
            self.package_num += 1
        fc.set_rgb_led(0, 0, 0)
        navi.set_height(self.cruise_height)
        navi.wait_for_height()
        logger.info(
            "[MISSION] Pack dropped, continue trajectory")

    def dxdy_to_fxfy(self):
        x, y, dx, dy = self.ori_hospital_pos
        fx, fy = transform_dxdy_to_fxfy_homography(dx, dy)
        fx_new=-fy
        fy_new=fx
        logger.info(f"fx:{fx_new},fy:{fy_new}")
        self.found_coordinates.append((fx_new+x, fy_new+y))

    def get_hospital_pos(self):
        if self.found_coordinates:
            # 成功找到的坐标数量（即进行平均计算的分母）
            count = len(self.found_coordinates)
            sum_x = 0
            sum_y = 0
            for x, y in self.found_coordinates:
                sum_x += x
                sum_y += y
            average_x = sum_x / count
            average_y = sum_y / count
            # 最终的平均坐标
            self.hospital_pos = [average_x, average_y]

            logger.info(f"成功找到的坐标数量: {count}")
            logger.info(f"所有找到的坐标: {self.found_coordinates}")
            logger.info(f"红十字坐标 : {self.hospital_pos}")
        else:
            logger.info("三次寻找操作中，没有找到任何坐标。无法计算平均坐标。")

    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()

    def camera_task(self):
        self.ob_count2 = 0
        while not self.find_hospital_mission_finish:
            ret, img = self.cam2.read()
            if not ret:
                logger.warning("2没有图像")
                index=get_usb_2_0_camera()
                logger.info(f"尝试重连摄像头 {index}...")
                # 释放旧资源 (如果存在)
                if self.cam2:
                    self.cam2.release()
                    self.cam2 = None # 清空旧对象
                for attempt in range(3):
                    time.sleep(0.1) # 等待 0.1 秒
                    self.cam2 = cv2.VideoCapture(index)
                    if self.cam2.isOpened():
                        ret, img = self.cam2.read()
                        if ret:
                            logger.info(f"摄像头 {index} 重连成功。")
                            break
                    self.cam2.release() # 释放失败的连接
                    logger.warning(f"摄像头 {index} 重连失败 (尝试 {attempt + 1}/{3})。")
                    if attempt == 2:
                        logger.error(f"摄像头 {index} 无法重连。")
                        return
            if self.find_hospital:
                f, _, _, _ = find_red_area(img, 1)
                debug_imshow(img, "find_red_area_ori")
                if f:
                    logger.info("2瞟到一眼红十字")
                    self.ob_count += 1
                else:
                    if self.ob_count:
                        logger.info("2刚才估计看错")
                    self.ob_count = 0
                    continue
                if self.ob_count > 0:  # 此处改确认存在次数
                    f, dx, dy, _ = find_red_area(img, 1)
                    if f:
                        x = self.navi.current_x
                        y = self.navi.current_y
                        logger.info(f"cur_x:{x},cur_y{y}")
                        self.ob_count = 0
                        self.find_hospital = False
                        self.found_hospital = True
                        logger.info("找到红十字了")
                        self.ori_hospital_pos = [
                            x, y, dx, dy]
                        logger.info(f"相机位置dx：{dx}，dy：{dy}")
                        self.dxdy_to_fxfy()
                    else:
                        logger.info("2刚才估计看错")
                        self.ob_count = 0
                        continue
            time.sleep(0.01)
        if self.cam2 is None or not isinstance(self.cam2, cv2.VideoCapture):
            logger.info("摄像头对象无效或未初始化。无需释放。")
            return
        if self.cam2.isOpened():
            logger.info("摄像头资源当前被占用，正在执行释放操作...")
            try:
                # 释放摄像头
                self.cam2.release()
                logger.info("摄像头资源释放成功。")
            except Exception as e:
                logger.info(f"释放摄像头时发生错误: {e}")
        else:
            logger.info("摄像头资源未被打开或已释放。无需操作。")
        time.sleep(0.5)
        ret, img = self.cam1.read()
        if not ret:
            logger.warning("1没有图像")
            index=get_sy_1080p_camera()
            logger.info(f"尝试重连摄像头 {index}...")
            # 释放旧资源 (如果存在)
            if self.cam1:
                self.cam1.release()
                self.cam1 = None # 清空旧对象
            for attempt in range(3):
                time.sleep(0.1) # 等待 0.1 秒
                self.cam1 = cv2.VideoCapture(index)
                if self.cam1.isOpened():
                    ret, img = self.cam1.read()
                    if ret:
                        logger.info(f"摄像头 {index} 重连成功。")
                        break
                self.cam1.release() # 释放失败的连接
                logger.warning(f"摄像头 {index} 重连失败 (尝试 {attempt + 1}/{3})。")
                if attempt == 2:
                    logger.error(f"摄像头 {index} 无法重连。")
                    return

        _, _, _, _ = self.identify_area(img)
        self.ob_count = 0
        self.timeout = 0
        navi = self.navi
        while True:
            ret, img = self.cam1.read()
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
                if self.timeout > 3:
                    logger.warning("此处没识别到任何")
                    self.identify_status = False
                    self.next_point_event.set()
            if self.id_hospital:
                debug_imshow(img, "Origin")
                f, _, _, res = find_red_area(img, 50)
                if f:
                    self.ob_count += 1
                    self.timeout = 0
                else:
                    self.ob_count = 0
                    self.timeout += 1
                if self.ob_count > 1:  # 此处改确认存在次数
                    self.ob_count = 0
                    self.timeout = 0
                    self.id_hospital = False
                    if self.started:
                        logger.info("红十字找到了")
                        time.sleep(0.1)
                        self.target_hospital()
                        self.next_point_event.set()
                if self.timeout > 3:
                    logger.warning("此处没找到红十字")
                    self.id_hospital = False
                    self.next_point_event.set()
            time.sleep(0.06)
            # print(f"瞬时FPS: {timer.fps():.1f}")
            timer.reset()

    def path_3d(self, path=None):

        height_column = np.ones((len(path), 1)) * HEIGHT
        path_3d = np.hstack((path, height_column))
        logger.info(f"path{path_3d}")
        return path_3d

    def find_door(self):

        MAX_ATTEMPTS = 5  # 设置最大尝试次数
        attempt_count = 0

        while attempt_count < MAX_ATTEMPTS:
            logger.info(f"正在进行第 {attempt_count + 1} 次门搜索...")
                # 获取最新点云
            img = radar.map.output_cloud()
            debug_imshow(img, "Origin")
            detected_walls = radar_resolve_wall(
                img, 180, 1, 2, 100, 20, True)
            num=len(detected_walls)
            if len(detected_walls) == 0:
                    logger.info("未检测到任何墙壁。")
            elif len(detected_walls) == 1:
                logger.info("检测到1条墙壁：")
                wall_angle, wall_distance, wall_segment = detected_walls[0]
                
                x1, y1, x2, y2 = wall_segment
                
                logger.info(f"  - 距离: {wall_distance:.2f} 像素, 角度: {wall_angle:.2f} 度")
                logger.info(
                    f"  - 转换后线段端点: ({x1:.2f}, {y1:.2f}) -> ({x2:.2f}, {y2:.2f})")
                midpoint_x = (x1 + x2) / 2
                midpoint_y = (y1 + y2) / 2
                
                return num,(midpoint_x,midpoint_y)
            elif len(detected_walls) >= 2:
                wall1_segment = detected_walls[0][2]
                wall2_segment = detected_walls[1][2]
            
                points = [
                    (wall1_segment[0], wall1_segment[1]),
                    (wall1_segment[2], wall1_segment[3]),
                    (wall2_segment[0], wall2_segment[1]),
                    (wall2_segment[2], wall2_segment[3])
                ]
                sorted_points = sorted(points, key=lambda p: p[1])
                middle_point1 = sorted_points[1]
                middle_point2 = sorted_points[2]

                logger.info("\n中间两个点的坐标是：")
                logger.info(f"  - 第一个中间点: ({middle_point1[0]:.2f}, {middle_point1[1]:.2f})")
                logger.info(f"  - 第二个中间点: ({middle_point2[0]:.2f}, {middle_point2[1]:.2f})")

                # 计算这两个点的中点
                midpoint_x = (middle_point1[0] + middle_point2[0]) / 2
                midpoint_y = (middle_point1[1] + middle_point2[1]) / 2

                logger.info(f"\n中点坐标是: ({midpoint_x:.2f}, {midpoint_y:.2f})")
                return num,(midpoint_x,midpoint_y)
            time.sleep(0.5)
             

        logger.info(f"已达到最大尝试次数 ({MAX_ATTEMPTS})，放弃搜索。")
        return None

    def next_mission(self):
        logger.info("开始穿越障碍区任务")
        step=0
        dis=100   #穿过墙的距离
        navi.set_navigation_speed(15)
        self.navi.navigation_to_waypoint(NEXT_MISSION_PATH[1],wait=True)
        self.navi.wait_for_waypoint(time_thres=0.5,pos_thres=8)
        for step in range(2):
            logger.info(f"第{step+1}道门")
            num,radar_point=self.find_door()
            if radar_point is not None:
                point=(radar_point[0]+navi.current_x,radar_point[1]+navi.current_y)
                logger.info(f"找到了门中点{point}")
                if num==1:
                    if point[1]>=NEXT_MISSION_POINT[1]:
                        navi.navigation_to_waypoint((navi.current_x,point[1]-70))
                        navi.wait_for_waypoint(time_thres=0.8,pos_thres=8)
                        navi.navigation_to_waypoint((point[0]-dis,point[1]-70))
                        navi.wait_for_waypoint(time_thres=0.8,pos_thres=8)
                    else    :
                        navi.navigation_to_waypoint((int(navi.current_x),point[1]+65))
                        navi.wait_for_waypoint(time_thres=0.8,pos_thres=8)
                        navi.navigation_to_waypoint((point[0]-dis,point[1]+65))
                        navi.wait_for_waypoint(time_thres=0.8,pos_thres=8)   
                else:
                    navi.navigation_to_waypoint((int(navi.current_x),point[1]))
                    navi.wait_for_waypoint(time_thres=0.8,pos_thres=8)
                    navi.navigation_to_waypoint((point[0]-dis,point[1]))
                    navi.wait_for_waypoint(time_thres=0.8,pos_thres=8)
            else :
                logger.info("失败了")
                navi.pointing_landing(navi.current_point)
                time.sleep(0.1)
            time.sleep(0.5)
        if step==0:
            navi.navigation_to_waypoint((int(navi.current_x-200),NEXT_MISSION_POINT[1]))
            navi.wait_for_waypoint(time_thres=0.8,pos_thres=8)
        if step==1:
            logger.info("穿越完成")
            navi.navigation_to_waypoint((int(navi.current_x-20),NEXT_MISSION_POINT[1]))
            navi.pointing_landing(navi.current_point)

    def add_hospital_optimized(self):
            """
            优化医院坐标的插入逻辑，使其插入到相邻任务点之间，
            使得总路程（P_i -> H -> P_{i+1}）增加最小，从而避免不必要的折返。
            """
            global MISSION_POINTS
            
            if not self.found_hospital:
                logger.warning("Error: Hospital not found. Skipping coordinate addition.")
                return
            hospital_x = int(self.hospital_pos[0] // 5 * 5)
            hospital_y = int(self.hospital_pos[1] // 5 * 5)
            hospital_point = np.array([hospital_x, hospital_y])
            
            logger.info(f"Hospital Final Coords: ({hospital_x}, {hospital_y})")
            if MISSION_POINTS.size == 0:
                MISSION_POINTS = hospital_point.reshape(1, -1)
                logger.info("MISSION_POINTS was empty. Hospital added as the first point.")
                self.closest_index = 0  # 记录插入位置
                return
            shortest_added_distance = float('inf')
            best_insert_index = -1  # 记录插入到哪个点“之前”
            points_list = [tuple(p) for p in MISSION_POINTS]
            
            # 0. 添加一个“虚拟起点”来计算插入到 MISSION_POINTS[0] 之前的情况
            start_point = self.navi.current_point
            
            extended_points = [start_point] + points_list
            
            num_points = len(extended_points)

            for i in range(num_points - 1):
                P_i = np.array(extended_points[i])
                P_i_plus_1 = np.array(extended_points[i+1])
                original_dist = np.linalg.norm(P_i - P_i_plus_1)
                new_dist = np.linalg.norm(P_i - hospital_point) + np.linalg.norm(hospital_point - P_i_plus_1)
                added_distance = new_dist - original_dist
                logger.debug(f"Segment {i} -> {i+1}: Original={original_dist:.2f}, New={new_dist:.2f}, Added={added_distance:.2f}")

                if added_distance < shortest_added_distance:
                    shortest_added_distance = added_distance
                    best_insert_index = i
            # best_insert_index 表示在 MISSION_POINTS 的第 best_insert_index 个元素之前插入。

            if best_insert_index != -1:
                insert_index_in_mission_points = best_insert_index
                
                hospital_point_to_insert = hospital_point.reshape(1, -1)
                
                is_hospital_already_near = False
                # 检查 P_{i} 和 P_{i-1}
                if insert_index_in_mission_points > 0:
                    prev_point = MISSION_POINTS[insert_index_in_mission_points - 1]
                    if np.linalg.norm(prev_point - hospital_point) < 5: # 距离小于5cm认为是同一个点
                        is_hospital_already_near = True
                if not is_hospital_already_near and insert_index_in_mission_points < len(MISSION_POINTS):
                    next_point = MISSION_POINTS[insert_index_in_mission_points]
                    if np.linalg.norm(next_point - hospital_point) < 5:
                        is_hospital_already_near = True
                if is_hospital_already_near:
                    logger.warning(f"Hospital point is too close to an existing mission point. Skipping insertion.")
                    return

                MISSION_POINTS = np.insert(
                    MISSION_POINTS,
                    insert_index_in_mission_points,
                    hospital_point_to_insert,
                    axis=0
                )
                self.closest_index = insert_index_in_mission_points # 更新索引
                logger.info(
                    f"Hospital added successfully at index {insert_index_in_mission_points}. Added distance: {shortest_added_distance:.2f}."
                )
                logger.info(f"New MISSION_POINTS: {MISSION_POINTS}")
            else:
                logger.warning(
                    "Failed to find best insertion index (Should not happen if array is not empty)."
                )

    def run(self):
        fc = self.fc
        cam1 = self.cam1
        cam2 = self.cam2
        navi = self.navi
        vision_debug(True) 
        self.navigation_speed = SPEED
        self.cruise_height = HEIGHT
        self.vertical_speed = 15

        self.inital_yaw = self.fc.state.yaw.value
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()
        navi.switch_navigation_mode("fusion-ros")
        navi.set_basepoint(BASE_POINT)
        navi.set_rs_speed_report(True, 2)
        fc.set_action_log(False)
       # set_cam_autowb(cam1, True)

        fc.set_indicator_led(0, 255, 0)
        time.sleep(0.05)
        # self.serial_terminal.listen_start(self.rxbuffer)
        self.serial_gpio.send_start(self.gpio_com)
        time.sleep(0.1)
        self.started = True
        for _ in range(3):
            time.sleep(0.25)
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(0.25)
            fc.set_indicator_led(0, 0, 0)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")
        navi.pointing_takeoff(BASE_POINT, 85)
        navi.wait_for_height()
        
        # time.sleep(0.5)
        # self.find_hospital = False
        # navi.navigation_to_waypoint((-100, 0), wait=True)
        # self.find_hospital = True


        # for points in MISSION_POINTS:
        #     logger.info(f"[MISSION] Go to point {points}")
        #     path = self.Astar_plan(
        #         [int(self.navi.current_x//5*5), int(self.navi.current_y//5*5)], points)
        #     logger.info("生成路径成功")
        #     if path is not None and len(path):
        #         path = self.path_3d(path)

        #     else:
        #         logger.warning(
        #             f"[MISSION] Path planning failed for point {points}. Skipping navigation.")
        #         break
        #     logger.info("kaishidaohang")
        #     navi.navigation_follow_trajectory(path, wait=True)
        #     self.next_point_event.clear()
        #     if self.found_hospital:
        #         if self.pos_id == self.closest_index:
        #             self.id_hospital = True
        #             logger.info("开始找红十字")
        #         else:
        #             self.identify_status = True
        #             logger.info("到点，识别")
        #     else :
        #         self.identify_status = True
        #         logger.info("到点，识别")
        #     self.next_point_event.wait()
        #     self.next_point_event.clear()
        #     self.pos_id += 1


        self.next_mission()
        time.sleep(0.1)


if __name__ == "__main__":
    fc = FC_Client()
    fc.connect()
    fc.wait_for_connection()
    t265 = T265("ros")
    t265.start()
    radar = LD_Radar()
    radar.start("ros")
    cam1 = cv2.VideoCapture(get_sy_1080p_camera())
    cam2 = cv2.VideoCapture(get_usb_2_0_camera())
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
        cam1=cam1,
        cam2=cam2,
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
