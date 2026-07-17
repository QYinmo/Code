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
from usr_serial_fc import Serial_station, Serial_gpio
from ultralytics import YOLO
from mission_2025_draft import xy_to_ab, id_to_xy
import vision_client_example    
timer = HighPrecisionFPS()
cfg = ConfigManager(section="mission")

BASE_CALI_POINT = cfg.get_array(
    "point-base", default=np.array([81.28021476, 418.3601995]))

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
MISSION_POINTS: np.ndarray = np.array([])
def P(x, y): return np.array([x * 50, y * 50, HEIGHT])

DT=0.2
X_SIZE = np.array([50, 0, 0])
Y_SIZE = np.array([0, 50, 0])
SPEED = 15
SPEED_LOW=10
HEIGHT = 105


ANIMAL_CLASS = {
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
        self.already = set((0,0))  # 记录已经查询过的点
        self.scope = 10  # 在多少范围内认为是同一个点
        self.allow_repeat = True  # 是否不需要查重
        self.conf_thres = 0.9
        self.area_count = 0
        self.repeat_num=0
        self.gpio_com = [170, 0, 255]  # laser
        self.terminal_com = [170, 0, 0, 0, 0, 0, 0, 0, 255]  # 当前位置a,b，象虎狼猴孔雀分别对应位置的数量
        self.terminal_rxbuffer = []
        if self.use_serial:
            self.serial_terminal = Serial_station(
                device="cp2102", baudrate=115200)
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

    def id_function(self):
        # 从服务获取字典格式的检测结果
        result = vision_client_example.get_detection_results(block=True, timeout=0.2)
        if result.get('success'):
            has_detection = result.get('has_detection', False)
            if has_detection:
                logger.info("检测到目标：")
                all_detected_objects = []
                for obj in result.get('detected_objects', []):
                    all_detected_objects.append([
                        float(obj['dx']),  # dx
                        float(obj['dy']),  # dy
                        int(obj['class_id']), # 类别
                        float(obj['confidence']) # 置信度
                    ])
                    logger.info(f" - 类别: {obj['class_name']}, 置信度: {obj['confidence']:.2f}, 位置: ({obj['dx']:.2f}, {obj['dy']:.2f})")
                # 计算类别统计
                class_counts = {}
                for obj in all_detected_objects:
                    class_id = obj[2]
                    class_counts[class_id] = class_counts.get(class_id, 0) + 1

                logger.info("\n--- 类别统计 ---")
                for cls_id, count in class_counts.items():
                    class_name = ANIMAL_CLASS.get(cls_id, f"Unknown_{cls_id}")
                    logger.info(f"类别 {class_name}: {count} 个")
                    
                return True, all_detected_objects, class_counts
            else:
                logger.info("未检测到任何目标")
                return False, [], {}
        else:
            logger.info(f"获取结果失败: {result.get('error', '未知错误')}")
            return False, [], {}


    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()

    def camera_task(self):
            self.timeout = 0
            navi = self.navi
            while True:
                if self.identify_status:
                    f, pos, statistic = self.id_function()
                    if f:
                        self.timeout = 0
                        if self.started:
                            logger.info("停下导航")
                            self.navi.navigation_stop_here()
                            time.sleep(0.05)
                            f, pos, statistic = self.id_function()
                            if not f:
                                continue
                            self.identify_status = False
                            self.record(pos, statistic)
                            self.navi.navigation_follow_trajectory(
                                self.navi.traj_list_before_stop, wait=False,dt=0.15)
                    else:
                        self.timeout += 1
                    if self.timeout > 3:
                        self.timeout = 0
                        self.identify_status = False
                        logger.info("此处没动物")
                        if not navi.traj_running_event.is_set():
                            self.navi.navigation_follow_trajectory(
                                self.navi.traj_list_before_stop, wait=False,dt=0.15)

                time.sleep(0.06)
                #logger.info(f"瞬时FPS: {timer.fps():.1f}")
                timer.reset()
# ------处理与任务-------
    def record(self, positon, statistic):  # 记录数据
        navi = self.navi
        start_point = np.array([navi.current_x, navi.current_y])
        logger.info(f"开始动物拍照任务，当前原点坐标为: ({start_point[0]:.2f}, {start_point[1]:.2f})")
        K = 10  # 实地考察计算
        for i, obj_info in enumerate(positon):
            dx = obj_info[0]
            dy = obj_info[1]
            class_id = obj_info[2]
            logger.info(f"目标 {i+1}:")
            logger.info(f"类别 ID: {class_id}")
            logger.info(f"dx,dy: ({dx:.5f}, {dy:.5f})")
            self.target(start_point, dx, dy)
        logger.info("所有动物照完了，准备返回原点。")
        # 所有动物位置都已处理完毕，现在返回到任务起点
        navi.direct_set_waypoint(start_point)
        navi.wait_for_waypoint(time_thres=0.5)
        logger.info("已返回原点，任务结束。")
        logger.info("\n--- 类别统计 ---")
        if statistic:  # 检查字典是否为空
            for cls_id, count in statistic.items():
                logger.info(f"类别 {cls_id}: 共 {count} 个")
                self.terminal_com[cls_id + 3] = count
        else:
            logger.info("没有可用的类别统计数据。")
    def target(self, start_point, dx, dy):  # 照动物
        K = 10
        navi = self.navi
        to_point = np.array([start_point[0] - dx / K, start_point[1] + dy / K])
        logger.info(f"正在飞往动物位置: x:{to_point[0]:.2f}, y:{to_point[1]:.2f}")
        # 飞到动物位置
        navi.direct_set_waypoint(to_point)
        navi.wait_for_waypoint(time_thres=0.5)
        logger.info("动物照完了")

    # def record(self, positon, statistic):  # 记录数据
    #     for i, obj_info in enumerate(positon):
    #         dx = obj_info[0]
    #         dy = obj_info[1]
    #         class_id = obj_info[2]
    #         logger.info(f"目标 {i+1}:")
    #         logger.info(f"  类别 ID: {class_id}")
    #         logger.info(f"  dx,dy: ({dx:.5f}, {dx:.5f})")
    #         self.target(dx, dy)

    # def target(self, dx, dy):  # 照动物
    #     K = 10  # 实地考察计算
    #     navi = self.navi
    #     point = navi.current_point
    #     logger.info(f"target当前坐标{point}")
    #     now_point=P(*self.points)
    #     to_point = (point[0] - dx / K, point[1] + dy / K)
    #     logger.info(f"x:{to_point[0]},y:{to_point[1]}")
    #     navi.direct_set_waypoint(to_point)
    #     navi.wait_for_waypoint(time_thres=0.5)
    #     navi.direct_set_waypoint(now_point)
    #     navi.wait_for_waypoint(time_thres=0.5)
    #     logger.info("下一个动物")
# -------run------
    def find_turn_points(self,path):
        """
        从一个坐标路径中提取所有转折点。

        参数:
        path (list): 包含坐标对（例如 [x, y]）的列表。

        返回:
        list: 包含所有转折点的列表。
        """
        if len(path) < 3:
            return []

        turn_points = []
        # 遍历路径，从第二个点到倒数第二个点
        for i in range(1, len(path) - 1):
            prev_point = path[i - 1]
            current_point = path[i]
            next_point = path[i + 1]

            # 计算前后两段路径的方向
            direction_prev_x = current_point[0] - prev_point[0]
            direction_prev_y = current_point[1] - prev_point[1]
            direction_next_x = next_point[0] - current_point[0]
            direction_next_y = next_point[1] - current_point[1]

            # 如果前后两段的方向不同，则当前点是一个转折点
            # 也就是 (direction_prev_x, direction_prev_y) != (direction_next_x, direction_next_y)
            if (direction_prev_x != direction_next_x) or (direction_prev_y != direction_next_y):
                turn_points.append(current_point)

        return turn_points

    def get_route(self):  # 根据串口信号转化为任务轨迹
        list_of_processed_arrays = []
        DT = 0.2
        mission_path: List[Tuple[float, float, float]] = []


        for element in self.terminal_rxbuffer[1:]:
            point = id_to_xy[element]
            single_np_array = P(*point)
            list_of_processed_arrays.append(single_np_array)
   
        if list_of_processed_arrays:
            final_stacked_array = np.stack(list_of_processed_arrays)
            

            path_list = [p.tolist() for p in final_stacked_array]
            turn_points = self.find_turn_points(path_list)
            # 将转折点列表转换为numpy数组以便后续比较
            turn_points_np = np.array(turn_points)

            ###找到45度降落起始位置###
            # 检查是否为直线并获取倒数第三个点
            last_point = final_stacked_array[-1]
            
            # 确保至少有3个点可以检查，并且最后一个点是 (0, 0)
            if len(final_stacked_array) >= 3 and np.allclose(last_point, np.array([0, 0,HEIGHT])):
                third_to_last_point = final_stacked_array[-3]
                
                vector1 = final_stacked_array[-2] - third_to_last_point
                vector2 = last_point - final_stacked_array[-2]

                cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]

                if np.isclose(cross_product, 0):
                    # 这些点共线（在一条直线上）
                    self.saved_point = third_to_last_point
                else:
                    # 这些点不共线（发生了转弯）
                    if np.allclose(final_stacked_array[-2], P(1, 0)):
                        self.saved_point = P(0, 2)
                    elif np.allclose(final_stacked_array[-2], P(0, 1)):
                        self.saved_point = P(2, 0)
                    else:
                        logger.info("路径错误") 

            for i in range(len(final_stacked_array) - 1):
                last_p = final_stacked_array[i]
                next_p = final_stacked_array[(i + 1)]
                length = np.linalg.norm(next_p - last_p)

                is_turn_point = False
                for turn_p in turn_points_np:
                    if np.allclose(next_p, turn_p):
                        is_turn_point = True
                        break
                
                if is_turn_point:
                    # 如果下一个点是转折点，使用低速
                    speed_to_use = SPEED_LOW
                    logger.info(f"在转折点 ({next_p[0]}, {next_p[1]}) 处使用低速 {SPEED_LOW}")
                else:
                    # 否则使用正常速度
                    speed_to_use = SPEED
                    logger.info(f"在 ({last_p[0]}, {last_p[1]}) 到 ({next_p[0]}, {next_p[1]}) 之间使用正常速度 {SPEED}")

                traj_g = TrajectoryGenerator(last_p, next_p, length / speed_to_use)
                traj_g.solve()
                t = 0.0
                while t < length / speed_to_use:
                    mission_path.append(traj_g.calc_position_xyz(t))
                    t += DT
        else:
            final_stacked_array = None
            logger.error("飞行路径不存在")
        return mission_path
    
    def land_45(self,land_point=(0,3)):
        DT=0.2
        land_path=np.array([
            P(*land_point),(0,0,10)

        ])
        plan_path=np.array([])
        for i in range(len(land_path) - 1):
            last_p = land_path[i]
            next_p = land_path[(i + 1)]
            length = np.linalg.norm(next_p - last_p)
            traj_g = TrajectoryGenerator(last_p, next_p, length / SPEED)
            traj_g.solve()
            t = 0.0
            while t < length / SPEED:
                plan_path.append(traj_g.calc_position_xyz(t))
                t += DT
        navi.navigation_follow_trajectory(land_path,wait=True,dt=0.15)
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
        navi = self.navi

        self.navigation_speed = 20
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
        self.start_camera_task()
        time.sleep(0.05)
        self.serial_terminal.start_transmit(
            self.terminal_com, self.terminal_rxbuffer)

        time.sleep(0.1)
        
        while True:
            if self.terminal_rxbuffer:
                if self.terminal_rxbuffer[0]==1:
                    logger.info("start fly")
                    break
            time.sleep(0.2)
        
        
        self.started = True
        self.status=0
        for _ in range(3):
            time.sleep(0.25)
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(0.25)
            fc.set_indicator_led(0, 0, 0)
        fc.set_action_log(True)
        mission_path=self.get_route()
        self.gpio_com[1] = 1
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        time.sleep(0.3)
        navi.navigation_follow_trajectory(mission_path, wait=False,dt=0.15)
        time_num=0
        while True:
            current_points=[navi.current_x,navi.current_y]
            self.points=(int((current_points[0]+25)//50),int((current_points[1]+25)//50))
            if self.points not in self.already :
                self.terminal_com[3:8] = [0]*5
                self.terminal_com[1] = xy_to_ab[self.points][0]
                self.terminal_com[2] = xy_to_ab[self.points][1]
                if np.sqrt((self.points[0]*50-current_points[0])**2+(self.points[1]*50-current_points[1])**2)<=30 :
                    self.status=1
                    self.already.add(self.points)
                    self.identify_status = True
                    logger.info("start identify")
            if np.sqrt((self.points[0]*50-current_points[0])**2+(self.points[1]*50-current_points[1])**2)>=30:
                self.status=0
                self.identify_status = False
            if self.points==(0,0)and time_num>=1000:
                logger("回到起点，准备降落")
                break
            time_num+=1
            time.sleep(0.15)
        self.land_45(self.saved_point)

        #################


if __name__ == "__main__":
    fc = FC_Client()
    fc.connect()
    fc.wait_for_connection()
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
    fc.set_digital_output(1, True)
    time.sleep(0.5)
    fc.set_digital_output(1, False)
    fc.set_indicator_led(0, 0, 0)
    time.sleep(1)
    fc.close()
