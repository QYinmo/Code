import time
import os
import time
import numpy as np
from FlightController import FC_Client, FC_Controller, FC_Like
from FlightController.Components import LD_Radar
from loguru import logger
from run_new import update_state, get_xyyaw_relative, get_route, calibrate
from SC import Controller, State
import cv2
import threading
from Vision_plus import find_red_area, set_cam_autowb, change_cam_resolution, set_manual_exporsure, vision_debug, debug_imshow  # open_camera
from usr_serial import Serial_gpio, Serial_station, get_radar_com
PATH = os.path.dirname(os.path.abspath(__file__))


def open_camera(cam_id=0, retries=3, warmup_frames=10):
    """带重试和预热的摄像头初始化"""
    for attempt in range(retries):
        cap = cv2.VideoCapture(cam_id)
        if not cap.isOpened():
            logger.error(f"尝试 {attempt+1}/{retries} 打开摄像头失败")
            continue

        # 强制设置基础参数（兼容大部分摄像头）
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 最小缓冲

        # 帧预热（必须步骤）
        for _ in range(warmup_frames):
            ret, _ = cap.read()
            if ret:
                logger.success(f"摄像头初始化成功，预热帧 #{_+1}")
                return cap
            time.sleep(0.1)

        cap.release()

    raise RuntimeError(f"无法初始化摄像头，已尝试 {retries} 次")


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.x0 = 1.172
        self.y0 = 3.369
        self.fc: FC_Like = kwargs["fc"]
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.radar: LD_Radar = kwargs["radar"]
        self.next_point_event = threading.Event()
        self.cam_lock = threading.Lock()       # 摄像头操作锁
        self.control_lock = threading.RLock()  # 控制指令锁
        self.data_lock = threading.Lock()      # 共享数据锁

        self.target_speed = 200
        self.dl = 0.03
        self.DT = 0.1
        self.stop_signal = 0
        self.identify_status = False
        self.lastdata = [0, 0]
        self.datalist = [[0, 0]]
        self.x0_fix = 0
        self.y0_fix = 0
        self.fire_time = 0
        self.use_serial = True
        self.flag = 0
        self.recv_bias = 5
        self.fire_time = 0
        self.state = 0
        self.gpio_com = [170, 0, 255]  # 镭射
        # 无人机位置x，y，里程mileage，是否火，火源位置x，y，火源街道，火源片区
        self.rxbuffer = [170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255]
        self.terminal_com = [170, 0, 255]
        self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)

    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()
        logger.info("摄像头工作")

    def target_fire(self, img):
        f, dx, dy, _ = find_red_area(img)
        if f:
            debug_imshow(img, "Origin")
            self.fire_time = 0
            logger.debug(f"dx:{dx}dy:{dy}")
            if dx < 20 and dx > -20:
                self.update_steer_and_speed(self.fc, 0, 0)
                logger.info(f"对准了")
                return 0, 1
            if dx <= -20:
                logger.warning(f"上票了")
                return 0, 0
            if dx >= 100:
                self.update_steer_and_speed(self.fc, 0, 40)
                return 1, 0
            if dx >= 20:
                self.update_steer_and_speed(self.fc, 0, 20)
                return 1, 0
        else:
            self.fire_time += 1
            self.update_steer_and_speed(self.fc, 0, 80)
            logger.info("还没找到")
        if self.fire_time >= 6:
            logger.warning(f"开的什么鸟车，毛没找到")
            self.fire_time = 0
            return 0, 0
        return 1

    def camera_task(self):
        try:
            while True:
                ret, img = self.cam.read()
                if not ret:
                    logger.warning("摄像头没东西")
                    break
                # img = img[50:200, 100:300]
                if self.identify_status:
                    while True:
                        logger.info("在看了")
                        ret, img = self.cam.read()
                        signal, self.state = self.target_fire(img)
                        if signal == 0:
                            break
                    self.identify_status = False
                    if self.state:
                        self.laser(3)
                        self.state = 0
                    self.next_point_event.set()
                cv2.waitKey()
        finally:
            self.cam.release()

    def laser(self, t):
        self.gpio_com[1] = 1
        logger.info("[MISSION] Laser ON")
        time.sleep(t)
        self.gpio_com[1] = 0
        logger.info("[MISSION] Laser OFF")

    def update_steer_and_speed(self, fc, steer_rad: float, speed_mps: float, speed_y: float = 0):
        WHEEL_PERIMETER = 0.21049
        def rpm(speed_mps): return speed_mps / WHEEL_PERIMETER * 60
        steer_deg = np.rad2deg(-steer_rad)
        fc.set_steer_and_speed(speed_mps, steer_rad)

    def gogogo(self, radar, fc, x0, y0, way):
        # x,y火源位置，way去还是回,0为去
        with self.control_lock:
            try:
                x, y, yaw = get_xyyaw_relative(radar)
                enter_p, leave_p = get_route(
                    x, y, x0, self.x0_fix, y0, self.y0_fix, dl=self.dl)

                vel = self.target_speed
                # current stat
                cx, cy, cyaw, ck = enter_p if not way else leave_p

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
                if way:
                    logger.info(f"finish")
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
        set_cam_autowb(cam, True)
        cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cam.set(cv2.CAP_PROP_EXPOSURE, 10)
        for _ in range(10):
            ret, _ = self.cam.read()
            if ret:
                logger.info("好了")
                break
            time.sleep(0.5)

        logger.info("[MISSION] Navigation started")
        self.start_camera_task()
        self.update_steer_and_speed(fc, 0, 0)
        calibrate(radar)
        vision_debug(saveimg=True)
        self.serial_gpio.send_start(self.gpio_com)
        time.sleep(2)
        ######### 任务开始########
        self.gogogo(radar=radar, fc=fc, x0=self.x0, y0=self.y0, way=0)
        time.sleep(1)
        self.next_point_event.clear()
        logger.info("开始识别")
        self.identify_status = True
        self.next_point_event.wait()
        self.next_point_event.clear()
        self.gogogo(radar=radar, fc=fc, x0=self.x0, y0=self.y0, way=1)


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

    cam = open_camera()

    logger.info(f"Camera  opened")
    hmi = None

    mission = Mission(fc=fc, radar=radar, cam=cam,
                      )  # type: ignore
    try:
        mission.run()
    except Exception as e:
        logger.exception(f"[MANAGER] Mission Failed")
    finally:
        mission.stop()

    logger.info("[MANAGER] Mission finished")
    fc.close()
