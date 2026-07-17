import time
import os
import time
import numpy as np
from FlightController import FC_Client, FC_Controller, FC_Like
from FlightController.Components import LD_Radar
from loguru import logger
from Lgui_new import tk_gui, cv_draw
from run_new import update_state, get_xyyaw_relative, get_route, calibrate
from SC import Controller, State
import cv2
import threading
from serial.tools.list_ports import comports
from Vision_plus import open_camera_plus, find_red_area, set_cam_autowb, change_cam_resolution, set_manual_exporsure, vision_debug
from usr_serial import Serial_gpio, Serial_station, get_radar_com
PATH = os.path.dirname(os.path.abspath(__file__))


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.radar: LD_Radar = kwargs["radar"]
        self.next_point_event = threading.Event()
        self.cam_lock = threading.Lock()       # 摄像头操作锁
        self.control_lock = threading.RLock()  # 控制指令锁
        self.data_lock = threading.Lock()      # 共享数据锁

        self.x0 = 0
        self.y0 = 0
        self.target_speed = 200
        self.dl = 0.03
        self.DT = 0.1
        self.gui = tk_gui()
        self.draw = cv_draw()
        self.stop_signal = 0
        self.identify_status = 0
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
        self.gpio_com = [170, 0, 255]  # 镭射
        # 无人机位置x，y，里程mileage，是否火，火源位置x，y，火源街道，火源片区
        self.rxbuffer = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.terminal_com = [170, 0, 255]
        if self.use_serial:
            self.serial_terminal = Serial_station(
                device="cp2102", baudrate=115200, rx_length=14)
            self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)

    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()

    def update_gui(self):
        if time.time() - self.timecount > 0.8:
            # 记录当前飞机位置为新的 lastxy，并同时更新 lastdata
            current_plane_x = self.rxbuffer[1]*2
            # 注意这里Y轴的符号反转，和 data_receive_task 中一样
            current_plane_y = self.rxbuffer[0]*2
            time.sleep(0.06)
            logger.info(
                f"fc：({current_plane_x},{current_plane_y})")
            # 计算距离
            distance = int(
                ((current_plane_x - self.lastxy[0]) ** 2 + (current_plane_y - self.lastxy[1]) ** 2) ** 0.5)
            self.totaldis += distance

            self.datalist.append(
                # 始终添加当前实际接收到的点
                [current_plane_x*self.recv_bias, current_plane_y*self.recv_bias])

            self.lastxy = [current_plane_x, current_plane_y]  # 更新 lastxy 为当前点

            self.gui.output_text1.configure(
                text=f"Total Distance:{self.totaldis}")
            if self.rxbuffer[3] == 1:
                self.gui.output_text2.configure(
                    text=f"Fire source location:{self.rxbuffer[5]},{self.rxbuffer[4]}")
            img = self.draw.show_draw_res(self.datalist)
            self.gui.show_img(img)
            self.timecount = time.time()

    def handle_gui(self):
        start_time = time.perf_counter()
        while True:
            if self.gui.ready_to_go is True and self.flag == 0:  # 只有当 ready_to_go 且任务未触发时才检查
                self.terminal_com[1] = 1
                # print("飞机go")
                if self.rxbuffer[3] == 1:
                    logger.info(
                        f"检测到火源：({self.rxbuffer[5]},{self.rxbuffer[4]})，开始执行任务。")
                    self.flag = 1  # 标记任务已触发

                    self.x0 = self.rxbuffer[5] / 50+0.352
                    self.y0 = self.rxbuffer[4] / 50+0.751
                    calibrate(self.radar)
                    time.sleep(0.5)
                    self.gogogo(radar=self.radar, fc=self.fc,
                                x0=self.x0, y0=self.y0, way=0)
                    time.sleep(1)
                    self.next_point_event.clear()
                    logger.info("开始识别")
                    self.identify_status = True
                    self.next_point_event.wait()
                    self.next_point_event.clear()
                    self.gogogo(radar=self.radar, fc=self.fc,
                                x0=self.x0, y0=self.y0, way=1)

                    self.gui.ready_to_go = False  # 或等待用户再次点击 Start 按钮
                if time.perf_counter()-start_time > 60:
                    logger.info(
                        f"检测到火源：(0.79,3.41)，开始执行任务。")
                    self.flag = 1  # 标记任务已触发

                    self.x0 = 0.79
                    self.y0 = 3.41
                    calibrate(self.radar)
                    time.sleep(0.5)
                    self.gogogo(radar=self.radar, fc=self.fc,
                                x0=self.x0, y0=self.y0, way=0)
                    time.sleep(1)
                    self.next_point_event.clear()
                    logger.info("开始识别")
                    self.identify_status = True
                    self.next_point_event.wait()
                    self.next_point_event.clear()
                    self.gogogo(radar=self.radar, fc=self.fc,
                                x0=self.x0, y0=self.y0, way=1)

                    self.gui.ready_to_go = False  # 或等待用户再次点击 Start 按钮

            time.sleep(0.2)  # 保持休眠，避免高CPU占用

    def main_loop(self):
        self.update_gui()
        self.gui.root.after(200, self.main_loop)

    def target_fire(self, img):
        f, dx, dy, _ = find_red_area(img)
        if f:
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
        cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        cam.set(cv2.CAP_PROP_EXPOSURE, 10)
        logger.info("[MISSION] Navigation started")
        vision_debug(saveimg=True)
        self.start_camera_task()
        self.serial_terminal.listen_start(
            self.rxbuffer)
        self.serial_gpio.send_start(self.gpio_com)
        threading.Thread(target=self.handle_gui, daemon=True).start()
        calibrate(radar)
        self.gui.root.after(50, self.main_loop)
        self.gui.gui_window()
        self.gui.root.mainloop()

        ######### 任务开始########


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
    cam = open_camera_plus(0)
    logger.info("Camera opened")
    mission = Mission(fc=fc, radar=radar, cam=cam)  # type: ignore
    logger.warning("Press Enter to start mission")
    try:
        mission.run()
    except Exception as e:
        logger.exception("[MANAGER] Mission Failed")
    finally:
        mission.stop()

    logger.info("[MANAGER] Mission finished")
    fc.close()
