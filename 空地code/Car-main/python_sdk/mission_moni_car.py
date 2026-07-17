import time
import os
import time
import numpy as np
from FlightController import FC_Client, FC_Controller, FC_Like
from FlightController.Components import LD_Radar
from loguru import logger
from mission_moni_run import update_state, get_xyyaw_relative, get_route, calibrate
from SC import Controller, State
import cv2
import threading
from serial.tools.list_ports import comports
from Vision_plus import debug_imshow, HighPrecisionFPS, get_ROI, find_anycolor_area, vision_debug, open_camera_plus, set_manual_exporsure
from usr_serial import Serial_gpio, Serial_station, get_radar_com
PATH = os.path.dirname(os.path.abspath(__file__))
timer = HighPrecisionFPS()


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.radar: LD_Radar = kwargs["radar"]
        self.next_point_event = threading.Event()
        self.cam_lock = threading.Lock()       # 摄像头操作锁
        self.control_lock = threading.RLock()  # 控制指令锁
        self.data_lock = threading.Lock()      # 共享数据锁
        self.count = 0
        self.timeout = 0
        self.identify_status = False
        self.last_res = -1
        self.started = False
        self.res = -1
        self.target_speed = 200
        self.dl = 0.05
        self.DT = 0.2
        self.stop_signal = 0
        self.timecount = 0
        self.totaldis = 0
        self.lastdata = [0, 0]
        self.datalist = [[0, 0]]
        self.lastxy = [0, 0]
        self.x0_fix = 0.02
        self.y0_fix = 0.02
        self.fire_time = 0
        self.use_serial = True
        self.flag = 0
        self.recv_bias = 1
        self.fire_time = 0
        self.state = 0
        self.gpio_com = [170, 1, 255]  # 电磁铁
        self.next_route_number = 1
        self.last_route_number = 0
        self.found = False
        self.finished = False
        self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)

    def route_plan(self):
        self.target_speed = 200
        if self.last_route_number in {1, 3}:  # 终点是B
            if self.found:
                self.next_route_number = 4
            else:
                self.next_route_number = 2
        elif self.last_route_number == 2:  # 终点是A
            if self.found:
                self.next_route_number = 5
            else:
                self.next_route_number = 3
                self.target_speed = 400
        elif self.last_route_number == 4:  # 终点在扔包处
            self.next_route_number = 6
        elif self.last_route_number == 5:
            self.next_route_number = 7
        else:
            logger.warning("路线错误")

    def id_function(self, img):
        logger.debug("正在识别")
        img = get_ROI(img, (0.2, 0, 0.8, 1))
        debug_imshow(img, "Origin")
        f, _, _, area = find_anycolor_area(img, np.array(
            [140, 255, 255]), np.array([100, 50, 40]), 80, False)
        if f:
            logger.info(f"area:{area}")
            return True, area
        logger.debug("没看到")
        return False, 0

    def camera_task(self):  # 到点后开识别
        self.count = 0
        self.timeout = 0
        while True:
            ret, img = self.cam.read()
            if not ret:
                logger.warning("没有图像")
                continue
            if self.identify_status:
                if self.last_route_number in {1, 3}:
                    time.sleep(2)
                    self.identify_status = False
                    self.next_point_event.set()
                    continue
                f, area = self.id_function(img)
                if f and area > 200:
                    self.timeout = 0
                    self.count += 1
                else:
                    self.timeout += 1
                if self.count > 3:  # 此处改识别次数
                    self.count = 0
                    self.timeout = 0
                    self.identify_status = False
                    if self.started:
                        self.res = int(area)
                        logger.info("做任务了")
                        self.do_task()
                        self.next_point_event.set()
                elif self.timeout > 15:  # 此处为超时次数
                    self.timeout = 0
                    self.count = 0
                    self.identify_status = False
                    if self.started:
                        self.res = -1
                        logger.info("识别超时")
                        self.next_point_event.set()

            time.sleep(0.06)
            print(f"瞬时FPS: {timer.fps():.1f}")
            timer.reset()

    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()

    def do_task(self):
        self.found = True
        self.route_plan()
        self.gogogo(self.radar, self.fc, self.next_route_number)
        logger.info("放东西了")
        self.finished = True
        time.sleep(1)
        self.gpio_com[1] = 0
        time.sleep(1)

    def update_steer_and_speed(self, fc, steer_rad: float, speed_mps: float, speed_y: float = 0):
        WHEEL_PERIMETER = 0.21049
        def rpm(speed_mps): return speed_mps / WHEEL_PERIMETER * 60
        steer_deg = np.rad2deg(-steer_rad)
        fc.set_steer_and_speed(speed_mps, steer_rad)

    def gogogo(self, radar, fc, way):
        # way路径编号
        with self.control_lock:
            try:
                x, y, yaw = get_xyyaw_relative(radar)
                route = get_route(x,  way, dl=self.dl)

                vel = self.target_speed
                # current stat
                cx, cy, cyaw, ck = route

                # x, y, yaw = 0, 0, 0
                initial_state = State(x, y, yaw, 0)
                mpst = Controller(cx, cy, cyaw, ck,
                                  self.target_speed, self.dl, initial_state)

                # fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.SPD_CTRL)
                update_state(radar, mpst)
                last_update = time.perf_counter()
                # for steer, acc in mpst.iter_output():
                #     print(steer, acc)

                for steer, acc in mpst.iter_output():
                    if self.stop_signal:
                        self.stop()
                        break
                    vel = mpst.get_speed() * 1.2
                    steer = steer * 1
                    # print(steer, vel)
                    self.update_steer_and_speed(fc, steer, vel)

                    logger.debug(f"steer: {steer}, acc: {acc}, vel: {vel}")
                    while time.perf_counter() - last_update < self.DT:
                        if self.stop_signal:
                            self.stop()
                            break
                        time.sleep(0.02)
                    last_update = time.perf_counter()
                    update_state(radar, mpst)

                self.update_steer_and_speed(fc, 0, 0)

            finally:
                self.update_steer_and_speed(fc, 0, 0)
                self.last_route_number = self.next_route_number
                time.sleep(0.5)

    def stop(self):
        self.update_steer_and_speed(self.fc, 0, 0)
        logger.info("[MISSION] Mission stopped")
        self.stop_signal = 0

    def run(self):
        radar = self.radar
        cam = self.cam
        fc = self.fc
        # change_cam_resolution(cam, 640, 480, 60)
        # set_cam_autowb(cam, True)
        set_manual_exporsure(cam, -8)
        logger.info("[MISSION] Navigation started")
        vision_debug(saveimg=True)
        self.start_camera_task()
        self.serial_gpio.send_start(self.gpio_com)
        calibrate(radar)
        ######### 任务开始########
        self.started = True
        while self.last_route_number not in {6, 7}:
            logger.info("出发咯")
            self.gogogo(radar, fc, self.next_route_number)
            if not self.finished:
                self.next_point_event.clear()
                self.identify_status = True
                self.next_point_event.wait()
                self.next_point_event.clear()
                self.route_plan()
            time.sleep(0.5)
        self.update_steer_and_speed(fc, 0, 0)
        logger.info("任务完成")


if __name__ == "__main__":
    logger.warning("DEBUG MODE!!")
    # fc = FC_Controller()
    # fc.start_listen_serial("/dev/ttyS6", print_state=True)
    fc = FC_Client()
    fc.connect()
    fc.wait_for_connection()
    t265 = None
    radar = LD_Radar()
    radar_port = get_radar_com()
    radar.start(radar_port, "LD06")
    radar.start(subtask_skip=5)
    radar.start_resolve_pose(800, 0.7, 0.3, rotation_adapt=True)
    cam = open_camera_plus()
    logger.info("Camera opened")
    mission = Mission(fc=fc, radar=radar, cam=cam)  # type: ignore
    try:
        mission.run()
    except Exception as e:
        logger.exception("[MANAGER] Mission Failed")
    finally:
        mission.stop()

    logger.info("[MANAGER] Mission finished")
    fc.close()
